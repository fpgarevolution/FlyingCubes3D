/***************************************************************
 * FPGA Revolution Open Bootcamp
 * Episode 34 - Flying cubes on FPGA
 *
 * Main application
 **************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "xil_printf.h"
#include "xil_types.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xaxidma.h"
#include "xscugic.h"
#include "cube.h"


#define GPIO_DEVICE_ID       XPAR_AXI_GPIO_0_DEVICE_ID

static XGpio gpio_ctl;                                    // GPIO interface driver instance
static XGpio_Config *gpio_cfg;                            // GPIO interface configuration parameters

#define DMA_DEVICE_ID        XPAR_AXI_DMA_0_DEVICE_ID
#define DMA_TRANSFER_SIZE  256                            // Each DMA transfer moves 256 32-bit words

static XAxiDma dma_ctl;                                   // AXI DMA driver instance
static XAxiDma_Config *dma_cfg;                           // AXI DMA configuration parameters

#define INTC_DEVICE_ID       XPAR_SCUGIC_0_DEVICE_ID      // Device ID of interrupt controller
#define INTC_INTERRUPT_ID    61                           // IRQ_F2P[0:0]


/* Interrupt handler function prototype */
void interrupt_handler (void *CallbackRef);

#define HRES  1280                                        // Display resolution in pixels
#define VRES  720

#define Z_OBJ                1                            // delta z of object
#define DISTANCE_OBJ         10.0                         // distance between object and screen
#define DISTANCE_OBJ_MIN     1.0                          // distance between object and screen
#define DISTANCE_SCREEN      5                            // distance between screen and camera

#define SLOPE_FIXEDPOINT_FRACTIONAL_BITS  16
#define SLOPE_MASK   0x0FFFFFFF                           // fixed-point slope uses lower 24 bits, 1.11.12

/* Triangle rotation function prototype */
static void triangle_rotation (const float p[3][3], bool);
/* Handle single flat-edge triangle function prototype */
static void flat_edge (float xy[3][2], bool flat_bottom);


static XScuGic_Config *intr_cfg;                          // Configuration parameters of interrupt controller
static XScuGic intr_ctl;                                  // Interrupt controller

static s32 status;
static bool interrupted = false;

#define TRI_ACTIVATE_MASK    0x80000000                   // Activate mask for a triangle

#define TRI_MAX              64                           // Max number of triangle pairs supported (1 pair = 1 flat-bottom + 1 flat-top)
#define DMA_WORD_PER_TRI     18                           // Number of 32-bit words required for each triangle pair for DMA
#define DMA_LENGTH           256*5                        // Number of 32-bit DMA transfers required to support the system, mininum to cover TRI_MAX * DMA_WORD_PER_TRI
#define RIGHT_ROT_MASK       0x80000000                   // Right rotate signal from PL
#define LEFT_ROT_MASK        0x40000000                   // Left rotate signal from PL
#define ANGLE_STEP           5.0                          // angle step per push button in degrees

static s32 data_dma_to_device[DMA_LENGTH];                  // Each DMA burst can be a portion of this buffer if required

// For flat-bottom triangle
static s32 tri_b_xy[TRI_MAX][3][2];                         // (x, y) vertices 
static s32 tri_b_dx[TRI_MAX][2];                            // dx of p1p2 and p1p3
static u32 color_b[TRI_MAX];                                // Fixed-intensity color
static u32 tri_color_b[TRI_MAX];                            // varying-intensity color
static u32 tri_b_index = 0;                                 // track number of flat-bottom triangles used
// For flat-top triangle
static u32 tri_t_xy[TRI_MAX][3][2];                         // (x, y) vertices 
static s32 tri_t_dx[TRI_MAX][2];                            // dx of p1p2 and p1p3
static u32 color_t[TRI_MAX];                                // fixed-intensity color
static u32 tri_color_t[TRI_MAX];                            // varying-intensity color
static u32 tri_t_index = 0;                                 // track number of flat-top triangles used

static float tri_intensity[TRI_MAX];

static float cube_offset = 0;                    
const static float z_center = DISTANCE_OBJ_MIN + DISTANCE_OBJ;
const static float radius = 8;
static float x_dyn = 0.0;
static float z_dyn = 0.0;
static float theta = 0.0;
static float theta_change = 0.5;
static float intensity = 1.0;
const static float rot_x = -25.0;
const static float rot_y = 35.0;
const static float rot_z = 0.0;

#define RED_MASK        0xFF0000
#define GREEN_MASK      0x00FF00
#define BLUE_MASK       0x0000FF


int main ()
{

	// Disable cache to prevent cache search forcing external memory access in this demonstration
	Xil_DCacheDisable();

	print("\nFPGA episode 34 - Flying Cubes on FPGA.\n\n");

	// Initialize GPIO driver
	gpio_cfg = XGpio_LookupConfig(GPIO_DEVICE_ID);
	if (NULL == gpio_cfg) {
		return XST_FAILURE;
	};
	status = XGpio_CfgInitialize(&gpio_ctl, gpio_cfg, gpio_cfg->BaseAddress);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Initialize AXI DMA driver
	dma_cfg = XAxiDma_LookupConfig(DMA_DEVICE_ID);
	if (NULL == dma_cfg) {
		return XST_FAILURE;
	}
	status = XAxiDma_CfgInitialize(&dma_ctl, dma_cfg);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
  //#define XAXIDMA_IRQ_IOC_MASK		0x00001000 /**< Completion intr */
  // Enable DMA-read interrupt
  XAxiDma_IntrEnable(&dma_ctl, XAXIDMA_IRQ_IOC_MASK, XAXIDMA_DMA_TO_DEVICE);

	// Initialize interrupt controller
	intr_cfg = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == intr_cfg) {
		return XST_FAILURE;
	}
	status = XScuGic_CfgInitialize(&intr_ctl, intr_cfg, intr_cfg->CpuBaseAddress);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Set interrupt priority and rising-edge trigger
	XScuGic_SetPriorityTriggerType(&intr_ctl, INTC_INTERRUPT_ID, 0xA0, 0x3);

	// Connect interrupt controller to ARM hardware interrupt handling logic
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler) XScuGic_InterruptHandler, &intr_ctl);
	// Enable ARM interrupts
	Xil_ExceptionEnable();

	// Connect interrupt handler
	status = XScuGic_Connect(&intr_ctl, INTC_INTERRUPT_ID, (Xil_InterruptHandler) interrupt_handler, NULL);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}


  u32 gap = 6;     // Cube has 1 square on each face = 2 triangular polygons

  /* Initialize all triangle activation */
  for (u32 i=0; i<TRI_MAX; i++) {
      // Flat-bottom triangle
      tri_b_dx[i][0] = 0;
      // Flat-top triangle
      tri_t_dx[i][0] = 0;

      if (i < gap) {
          // red
          color_b[i] = 0xff0000;
          color_t[i] = 0xff0000;
      } else if (i < gap*2) {
          // green
          color_b[i] = 0x00ff00;
          color_t[i] = 0x00ff00;
      } else if (i < gap*3) {
          // blue
          color_b[i] = 0x0000ff;
          color_t[i] = 0x0000ff;
      } else if (i < gap*4) {
          // yellow
          color_b[i] = 0xffff00;
          color_t[i] = 0xffff00;
      } else if (i < gap*5) {
          // white
          color_b[i] = 0xffffff;
          color_t[i] = 0xffffff;
      } else if (i < gap*6) {
          // magenta
          color_b[i] = 0xff00ff;
          color_t[i] = 0xff00ff;
      } else {
          // dark
          color_b[i] = 0x000000;
          color_t[i] = 0x000000;
      }

      // Initialize varying-intensity color to fixed
      tri_color_b[i] = color_b[i];
      tri_color_t[i] = color_t[i];

      // Initialize intensity of polys
      tri_intensity[i] = 1.0;
  }

	// Initialize DMA-read data buffer
	for (u32 i=0; i<DMA_LENGTH; i++) {
		data_dma_to_device[i] = 0;
	}

  for (u32 i=0; i<TRI_MAX; i++) {
      // Flat-bottom triangle 0
      data_dma_to_device[i*DMA_WORD_PER_TRI]     = tri_b_xy[i][0][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI + 1] = tri_b_xy[i][0][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI + 2] = tri_b_xy[i][1][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI + 3] = tri_b_xy[i][1][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI + 4] = tri_b_xy[i][2][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI + 5] = tri_b_xy[i][2][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI + 6] = tri_b_dx[i][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI + 7] = tri_b_dx[i][1];
      data_dma_to_device[i*DMA_WORD_PER_TRI + 8] = tri_color_b[i];
      // Flat-top triangle 0
      data_dma_to_device[i*DMA_WORD_PER_TRI + 9] = tri_t_xy[i][0][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI +10] = tri_t_xy[i][0][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI +11] = tri_t_xy[i][1][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI +12] = tri_t_xy[i][1][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI +13] = tri_t_xy[i][2][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI +14] = tri_t_xy[i][2][1]; 
      data_dma_to_device[i*DMA_WORD_PER_TRI +15] = tri_t_dx[i][0];
      data_dma_to_device[i*DMA_WORD_PER_TRI +16] = tri_t_dx[i][1];
      data_dma_to_device[i*DMA_WORD_PER_TRI +17] = tri_color_t[i];


      // Reset global index and activation after DMA
      // Flat-bottom triangle
      tri_b_dx[i][0] = 0;
      tri_b_index = 0;
      // Flat-top triangle
      tri_t_dx[i][0] = 0;
      tri_t_index = 0;
  }


	// Start DMA operation to move 3D rasterization info to PL for rendering
  // DMA 0
	status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*0], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};
  printf("DMA 0 finished.\n");

  // DMA 1
	status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*1], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};
  printf("DMA 1 finished.\n");

  // DMA 2
	status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*2], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};
  printf("DMA 2 finished.\n");

  // DMA 3
	status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*3], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};
  printf("DMA 3 finished.\n");

  // DMA 4
	status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*4], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};
  printf("DMA 4 finished.\n");

	// Enable device interrupt
	XScuGic_Enable(&intr_ctl, INTC_INTERRUPT_ID);


  bool request = false;
	// Sleep loop, interrupts have the spotlight
	while (TRUE) {
      if (interrupted) {

          interrupted = false;

          // Some debug hook along
          if (true) {
	            // Start timer
	            u32 gpio_latch = XGpio_DiscreteRead(&gpio_ctl, 2);                            

              // Speed change request
              if (gpio_latch & RIGHT_ROT_MASK) {
                  request = true;

                  theta_change = theta_change * 2;
              } else if (gpio_latch & LEFT_ROT_MASK) {
                  request = true;

                  theta_change = theta_change / 2;
              }

              // Force rendering with constant speed
              request = true; 


              // Render if told 
              if (request) {

                  if (theta >= 360.0) {
                      theta = theta - 360.0;
                  } else {
                      theta = theta + theta_change;
                  }

                  x_dyn = radius * cos(theta * M_PI / 180.0);
                  z_dyn = radius * sin(theta * M_PI / 180.0);
                  request = false;
                  

                  // Re-render cube 1
                  triangle_rotation(T1, true);
                  triangle_rotation(T2, false);
                  triangle_rotation(T3, true);
                  triangle_rotation(T4, false);
                  triangle_rotation(T5, true);
                  triangle_rotation(T6, false);
                  
                  // Re-render remaining cubes
                  for (u32 i=1; i<6; i++) {
                      // Cube 2
                      float theta2 = theta + 60*i;
                      if (theta2 >= 360) {
                          theta2 = theta2 - 360.0;
                      }

                      x_dyn = radius * cos(theta2 * M_PI / 180.0);
                      z_dyn = radius * sin(theta2 * M_PI / 180.0);

                      triangle_rotation(T1, true);
                      triangle_rotation(T2, false);
                      triangle_rotation(T3, true);
                      triangle_rotation(T4, false);
                      triangle_rotation(T5, true);
                      triangle_rotation(T6, false);
                  }

                  // Update data buffers with 3D position changes accordingly
                  for (u32 i=0; i<TRI_MAX; i++) {
                      // Flat-bottom triangle 0
                      data_dma_to_device[i*DMA_WORD_PER_TRI]     = tri_b_xy[i][0][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 1] = tri_b_xy[i][0][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 2] = tri_b_xy[i][1][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 3] = tri_b_xy[i][1][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 4] = tri_b_xy[i][2][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 5] = tri_b_xy[i][2][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 6] = tri_b_dx[i][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 7] = tri_b_dx[i][1];
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 8] = tri_color_b[i];
                      // Flat-top triangle 0
                      data_dma_to_device[i*DMA_WORD_PER_TRI + 9] = tri_t_xy[i][0][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI +10] = tri_t_xy[i][0][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI +11] = tri_t_xy[i][1][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI +12] = tri_t_xy[i][1][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI +13] = tri_t_xy[i][2][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI +14] = tri_t_xy[i][2][1]; 
                      data_dma_to_device[i*DMA_WORD_PER_TRI +15] = tri_t_dx[i][0];
                      data_dma_to_device[i*DMA_WORD_PER_TRI +16] = tri_t_dx[i][1];
                      data_dma_to_device[i*DMA_WORD_PER_TRI +17] = tri_color_t[i];

                      // Reset global index and activation after DMA
                      // Flat-bottom triangle
                      tri_b_dx[i][0] = 0;
                      tri_b_index = 0; 
                      // Flat-top triangle
                      tri_t_dx[i][0] = 0;
                      tri_t_index = 0; 
                  }

                  // Kick off DMA to move updated 3D info
                  // DMA 0
	                status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*0], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	                if (status != XST_SUCCESS) {
	                	return XST_FAILURE;
	                }
	                while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};

                  // DMA 1
	                status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*1], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	                if (status != XST_SUCCESS) {
	                	return XST_FAILURE;
	                }
	                while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};

                  // DMA 2
	                status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*2], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	                if (status != XST_SUCCESS) {
	                	return XST_FAILURE;
	                }
	                while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};

                  // DMA 3
	                status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*3], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	                if (status != XST_SUCCESS) {
	                	return XST_FAILURE;
	                }
	                while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};

                  // DMA 4
	                status = XAxiDma_SimpleTransfer(&dma_ctl, (UINTPTR) &data_dma_to_device[DMA_TRANSFER_SIZE*4], DMA_TRANSFER_SIZE*4, XAXIDMA_DMA_TO_DEVICE);
	                if (status != XST_SUCCESS) {
	                	return XST_FAILURE;
	                }
	                while(XAxiDma_Busy(&dma_ctl, XAXIDMA_DMA_TO_DEVICE)) {};

              }

          }

          // Enable Zynq interrupt line 
          XScuGic_Enable(&intr_ctl, INTC_INTERRUPT_ID);

      }
	}


	return XST_SUCCESS;
}

/* Interrupt handler */
void interrupt_handler (void *CallbackRef) {
    // Disable Zynq interrupt line
    XScuGic_Disable(&intr_ctl, INTC_INTERRUPT_ID);

    interrupted = true;
}


/* Triangle rotation */
static void triangle_rotation (const float p[3][3], bool update_intensity) {

    float tri_0_vertex[3][2];
    float xr, yr, zr, xp, yp;
    bool flat_bottom = false;

    // For normal compute
    float P1_x, P1_y, P1_z;
    float P2_x, P2_y, P2_z;
    float P3_x, P3_y, P3_z;

    for (u32 i=0; i<3; i++) {
        //float x1 = p[i][0];
        float x1 = p[i][0] + x_dyn;
        float y1 = p[i][1];
        float z1 = p[i][2] + z_dyn;

        for (u32 j=0; j<3; j++) {
            if (j==0) {
                // around x axis
                xr = x1;
                yr = y1*cos(rot_x*M_PI/180.0) - z1*sin(rot_x*M_PI/180.0);
                zr = z1*cos(rot_x*M_PI/180.0) + y1*sin(rot_x*M_PI/180.0);

                x1 = xr; y1 = yr; z1 = zr;
            } else if (j==1) {
                // +around y axis
                xr = x1*cos(rot_y*M_PI/180.0) + z1*sin(rot_y*M_PI/180.0);
                yr = y1;
                zr = z1*cos(rot_y*M_PI/180.0) - x1*sin(rot_y*M_PI/180.0);

                x1 = xr; y1 = yr; z1 = zr;
            } else if (j==2) {
                // +around z axis
                xr = x1*cos(rot_z*M_PI/180.0) - y1*sin(rot_z*M_PI/180.0);
                yr = y1*cos(rot_z*M_PI/180.0) + x1*sin(rot_z*M_PI/180.0);
                zr = z1;

                x1 = xr; y1 = yr; z1 = zr;
            }

        }

        xp = xr * (DISTANCE_SCREEN / (DISTANCE_SCREEN + z_center + zr));
        yp = yr * (DISTANCE_SCREEN / (DISTANCE_SCREEN + z_center + zr));
 
        tri_0_vertex[i][0] = xp;
        tri_0_vertex[i][1] = yp;

        // For normal compute
        if (i==0) {
            P1_x = x1;
            P1_y = y1;
            P1_z = z1;
        } else if (i==1) {
            P2_x = x1;
            P2_y = y1;
            P2_z = z1;
        } else {
            P3_x = x1;
            P3_y = y1;
            P3_z = z1;
        }
    }

    // Compute normal vector
    float vP1P2[3] = {P2_x-P1_x, P2_y-P1_y, P2_z-P1_z};
    float vP1P3[3] = {P3_x-P1_x, P3_y-P1_y, P3_z-P1_z};

    float vNorm_x = vP1P2[1]*vP1P3[2] - vP1P2[2]*vP1P3[1];
    float vNorm_y = -(vP1P2[0]*vP1P3[2] - vP1P2[2]*vP1P3[0]);
    float vNorm_z = vP1P2[0]*vP1P3[1] - vP1P2[1]*vP1P3[0];

    vNorm_x = -vNorm_x;
    vNorm_y = -vNorm_y;
    vNorm_z = -vNorm_z;

    float scalar = vNorm_z*(-DISTANCE_SCREEN) / ( sqrt((-DISTANCE_SCREEN)*(-DISTANCE_SCREEN)) * sqrt(vNorm_x*vNorm_x + vNorm_y*vNorm_y + vNorm_z*vNorm_z));

    float theta = acos(scalar) * 180.0 / M_PI;


    float p1pe_x = 0 - P1_x;
    float p1pe_y = 0 - P1_y;
    float p1pe_z = -DISTANCE_SCREEN - P1_z;

    // Angle between normal vector and vector p1pe
    scalar = (p1pe_x*vNorm_x + p1pe_y*vNorm_y + p1pe_z*vNorm_z) / 
                      ( sqrt(p1pe_x*p1pe_x + p1pe_y*p1pe_y + p1pe_z*p1pe_z) * sqrt(vNorm_x*vNorm_x + vNorm_y*vNorm_y + vNorm_z*vNorm_z) );

    theta = acos(scalar) * 180.0 / M_PI;

    // Adjust color intensity based on the angle between the normal vector of the poly and the viewpoint vector
    // The smaller the angle, the stronger the intensity
    // 0 degrees => strongest intensity (1)
    // 90 degress => weakest intensity (0)

    if (update_intensity) {
        if (theta > 180.0) {
            intensity = 0.0;
        } else if (theta < 0.0) {
            intensity = 1;
        } else {
            intensity = (180.0 - theta) / 180.0;
        }

    }


    // Determine the kind of triangle this is
    float x1_b, y1_b;                            // Decomposed flat-bottom triangle
    float x2_b, y2_b;
    float x3_b, y3_b;

    float x1_t, y1_t;                            // Decomposed flat-top triangle
    float x2_t, y2_t;
    float x3_t, y3_t;

    float x1_tmp;
    float y1_tmp;
    float x2_tmp;
    float y2_tmp;
    float x3_tmp;
    float y3_tmp;

    float x1 = tri_0_vertex[0][0];
    float y1 = tri_0_vertex[0][1];
    float x2 = tri_0_vertex[1][0];
    float y2 = tri_0_vertex[1][1];
    float x3 = tri_0_vertex[2][0];
    float y3 = tri_0_vertex[2][1];


    // Decompose into flat-edge triangles
    if ((y1 != y2) && (y1 != y3) && (y2 != y3)) {
        // Rearrange points, '1' is middle point, '2' is on top and '3' is on the bottom
        if ((y1 < y2) && (y1 > y3)) {
            x1_tmp = x1; y1_tmp = y1;     
            x2_tmp = x2; y2_tmp = y2;     
            x3_tmp = x3; y3_tmp = y3;     
        } else if ((y1 < y3) && (y1 > y2)) {
            x1_tmp = x1; y1_tmp = y1;     
            x2_tmp = x3; y2_tmp = y3;     
            x3_tmp = x2; y3_tmp = y2;     
        } else if ((y2 < y1) && (y2 > y3)) {
            x1_tmp = x2; y1_tmp = y2;     
            x2_tmp = x1; y2_tmp = y1;     
            x3_tmp = x3; y3_tmp = y3;     
        } else if ((y2 < y3) && (y2 > y1)) {
            x1_tmp = x2; y1_tmp = y2;     
            x2_tmp = x3; y2_tmp = y3;     
            x3_tmp = x1; y3_tmp = y1;     
        } else if ((y3 < y1) && (y3 > y2)) {
            x1_tmp = x3; y1_tmp = y3;     
            x2_tmp = x1; y2_tmp = y1;     
            x3_tmp = x2; y3_tmp = y2;     
        } else if ((y3 < y2) && (y3 > y1)) {
            x1_tmp = x3; y1_tmp = y3;     
            x2_tmp = x2; y2_tmp = y2;     
            x3_tmp = x1; y3_tmp = y1;     
        }

        // Decompose
        if (x2_tmp == x3_tmp) {
            // Special case, vertical line p2->p3
            if (x1_tmp < x2_tmp) { 
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle
                x2_b = x1_tmp; y2_b = y1_tmp;
                x3_b = x2_tmp; y3_b = y1_tmp;

                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            } else {
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle
                x2_b = x2_tmp; y2_b = y1_tmp;
                x3_b = x1_tmp; y3_b = y1_tmp;

                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            }
        } else if (x2_tmp < x3_tmp) {
            if (x1_tmp <= x2_tmp) {
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle
                x2_b = x1_tmp; y2_b = y1_tmp;
                x3_b = x3_tmp - ((y3_tmp - y1_tmp) / (y3_tmp - y2_tmp) * (x3_tmp - x2_tmp));   y3_b = y2_b;

                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            } else if (x1_tmp >= x3_tmp) {                                                                                                      
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle                                     
                x3_b = x1_tmp; y3_b = y1_tmp;                                                                     
                x2_b = x2_tmp + ((y1_tmp - y2_tmp) / (y3_tmp - y2_tmp) * (x3_tmp - x2_tmp));   y2_b = y3_b;    
                                                                                                                       
                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            } else {                                                                                          
                // middle
                // Need to figure out if p1 is above or below the p2p3 diagonal slope line (negative slope)
                float slope_p2p3 = (y3_tmp - y2_tmp) / (x3_tmp - x2_tmp);
                float slope_p2p1 = (y1_tmp - y2_tmp) / (x1_tmp - x2_tmp);

                if (slope_p2p1 < slope_p2p3) {
                    // below
                    x1_b = x2_tmp; y1_b = y2_tmp;                     // flat-bottom triangle                                     
                    x2_b = x1_tmp; y2_b = y1_tmp;                                                                     

                    float temp = (y3_tmp - y1_tmp) / (y1_tmp - y2_tmp);
                    x3_b = x3_tmp - (temp * (x3_tmp-x2_tmp) * (1.0 / (1 + temp)));   y3_b = y2_b;    
                                                                                                                           
                    x1_t = x3_tmp; y1_t = y3_tmp;                     // flat-top triangle
                    x2_t = x2_b;   y2_t = y2_b;
                    x3_t = x3_b;   y3_t = y3_b;
                } else {
                    // above
                    x1_b = x2_tmp; y1_b = y2_tmp;                     // flat-bottom triangle                                     
                    x3_b = x1_tmp; y3_b = y1_tmp;                                                                     

                    float temp = (y2_tmp - y1_tmp) / (y1_tmp - y3_tmp);
                    x2_b = x2_tmp + (temp * (x3_tmp-x2_tmp) * (1.0 / (1 + temp)));   y2_b = y3_b;    
                                                                                                                           
                    x1_t = x3_tmp; y1_t = y3_tmp;                     // flat-top triangle
                    x2_t = x2_b;   y2_t = y2_b;
                    x3_t = x3_b;   y3_t = y3_b;
                }
            }
        } else if (x2_tmp > x3_tmp) {
            if (x1_tmp <= x3_tmp) {
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle
                x2_b = x1_tmp; y2_b = y1_tmp;
                x3_b = x2_tmp - ((y1_tmp - y2_tmp) / (y3_tmp - y2_tmp) * (x2_tmp - x3_tmp));   y3_b = y2_b;

                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            } else if (x1_tmp >= x2_tmp) {
                x1_b = x2_tmp; y1_b = y2_tmp;                         // flat-bottom triangle
                x3_b = x1_tmp; y3_b = y1_tmp;
                x2_b = x3_tmp + ((y3_tmp - y1_tmp) / (y3_tmp - y2_tmp) * (x2_tmp - x3_tmp));   y2_b = y3_b;

                x1_t = x3_tmp; y1_t = y3_tmp;                         // flat-top triangle
                x2_t = x2_b;   y2_t = y2_b;
                x3_t = x3_b;   y3_t = y3_b;
            } else {
                // middle
                // Need to figure out if p1 is above or below the p3p2 diagonal slope line (positive slope)
                float slope_p3p2 = (y2_tmp - y3_tmp) / (x2_tmp - x3_tmp);
                float slope_p3p1 = (y1_tmp - y3_tmp) / (x1_tmp - x3_tmp);

                if (slope_p3p1 < slope_p3p2) {
                    // below

                    x1_b = x2_tmp; y1_b = y2_tmp;                     // flat-bottom triangle
                    x3_b = x1_tmp; y3_b = y1_tmp;

                    float temp = (y3_tmp - y1_tmp) / (y1_tmp - y2_tmp);
                    x2_b = x3_tmp + (temp * (x2_tmp-x3_tmp) * (1.0 / (1 + temp)));   y2_b = y3_b;    

                    x1_t = x3_tmp; y1_t = y3_tmp;                     // flat-top triangle
                    x2_t = x2_b;   y2_t = y2_b;
                    x3_t = x3_b;   y3_t = y3_b;
                } else {
                    // above
                    x1_b = x2_tmp; y1_b = y2_tmp;                     // flat-bottom triangle
                    x2_b = x1_tmp; y2_b = y1_tmp;

                    float temp = (y2_tmp - y1_tmp) / (y1_tmp - y3_tmp);
                    x3_b = x2_tmp - (temp * (x2_tmp-x3_tmp) * (1.0 / (1 + temp)));   y3_b = y2_b;    

                    x1_t = x3_tmp; y1_t = y3_tmp;                     // flat-top triangle
                    x2_t = x2_b;   y2_t = y2_b;
                    x3_t = x3_b;   y3_t = y3_b;
                }
            }
        } 

        // sort out flat-bottom triangle (update results to global variables before DMAing to hardware)
        float combine_b[3][2] = {{x1_b, y1_b}, {x2_b, y2_b}, {x3_b, y3_b}};
        flat_edge(combine_b, true);      

        // sort out flat-top triangle (update results to global variables before DMAing to hardware)
        float combine_t[3][2] = {{x1_t, y1_t}, {x2_t, y2_t}, {x3_t, y3_t}};
        flat_edge(combine_t, false);      

    // flat-bottom detection, rearrange vertices (p1 on top, p2 bottom left, p3 bottom right)
    } else {
        if ((y1 == y2) && (y3>y1)) {
            flat_bottom = true;

            x1_tmp = x3; 
            y1_tmp = y3;
            if (x1 < x2) {
                x2_tmp = x1;
                y2_tmp = y1;
                x3_tmp = x2;
                y3_tmp = y2;
            } else {
                x2_tmp = x2;
                y2_tmp = y2;
                x3_tmp = x1;
                y3_tmp = y1;
            }
        } else if ((y1 == y3) && (y2>y1)) {
            flat_bottom = true;
    
            x1_tmp = x2; 
            y1_tmp = y2;
            if (x1 < x3) {
                x2_tmp = x1;
                y2_tmp = y1;
                x3_tmp = x3;
                y3_tmp = y3;
            } else {
                x2_tmp = x3;
                y2_tmp = y3;
                x3_tmp = x1;
                y3_tmp = y1;
            }
        } else if ((y2 == y3) && (y1>y2)) {
            flat_bottom = true;
    
            x1_tmp = x1; 
            y1_tmp = y1;
            if (x2 < x3) {
                x2_tmp = x2;
                y2_tmp = y2;
                x3_tmp = x3;
                y3_tmp = y3;
            } else {
                x2_tmp = x3;
                y2_tmp = y3;
                x3_tmp = x2;
                y3_tmp = y2;
            }
        // flat-top detection
        } else if ((y1 == y2) && (y3<y1)) {
    
            x1_tmp = x3; 
            y1_tmp = y3;
            if (x1 < x2) {
                x2_tmp = x1;
                y2_tmp = y1;
                x3_tmp = x2;
                y3_tmp = y2;
            } else {
                x2_tmp = x2;
                y2_tmp = y2;
                x3_tmp = x1;
                y3_tmp = y1;
            }
        } else if ((y1 == y3) && (y2<y1)) {
    
            x1_tmp = x2; 
            y1_tmp = y2;
            if (x1 < x3) {
                x2_tmp = x1;
                y2_tmp = y1;
                x3_tmp = x3;
                y3_tmp = y3;
            } else {
                x2_tmp = x3;
                y2_tmp = y3;
                x3_tmp = x1;
                y3_tmp = y1;
            }
        } else if ((y2 == y3) && (y1<y2)) {
    
            x1_tmp = x1; 
            y1_tmp = y1;
            if (x2 < x3) {
                x2_tmp = x2;
                y2_tmp = y2;
                x3_tmp = x3;
                y3_tmp = y3;
            } else {
                x2_tmp = x3;
                y2_tmp = y3;
                x3_tmp = x2;
                y3_tmp = y2;
            }
        }

        // sort out flat-edge triangle (update results to global variables before DMAing to hardware)
        float combine[3][2] = {{x1_tmp, y1_tmp}, {x2_tmp, y2_tmp}, {x3_tmp, y3_tmp}};
        flat_edge(combine, flat_bottom);      

    }
}


/* Handle single flat-edge triangle */
static void flat_edge (float xy[3][2], bool flat_bottom) {

    float dx1, dx2;

    float hres_h = HRES / 2;                                   
    float hres_q = HRES / 4;                                   
    float vres_h = VRES / 2;                                   
    float vres_q = VRES / 4;                                   

    float r, g, b;
    u32 r32, g32, b32;

    if (flat_bottom) {
        dx1 = (xy[1][0] - xy[0][0]) / (xy[0][1] - xy[1][1]);                // x(p2-p1) / y(p1-p2)
        dx2 = (xy[2][0] - xy[0][0]) / (xy[0][1] - xy[2][1]);                // x(p3-p1) / y(p1-p3)

        // Convert to screen pixel coordinates
        tri_b_dx[tri_b_index][0] = ((s32) (dx1 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;
        tri_b_dx[tri_b_index][0] |= TRI_ACTIVATE_MASK;    // activate this triangle in hardware
        tri_b_dx[tri_b_index][1] = ((s32) (dx2 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;

        for (u32 i=0; i<3; i++) {
            float temp1 = xy[i][0] * vres_q + hres_h + cube_offset;                    // x, shift right to center
            float temp2 = -(xy[i][1] * vres_q) + vres_h;                               // y, different method, scale and subtract
            tri_b_xy[tri_b_index][i][0] = ((s32) (temp1 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;       // x, shift right to center
            tri_b_xy[tri_b_index][i][1] = ((s32) (temp2 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;       // y, different method, scale and subtract
        } 

        // Adjust RGB component based on updated intensity
        r = (color_b[tri_b_index] >> 16) & 0xFF;
        g = (color_b[tri_b_index] >>  8) & 0xFF;
        b = (color_b[tri_b_index] >>  0) & 0xFF;

        r = r * intensity;
        g = g * intensity;
        b = b * intensity;

        r32 = (((u32) r) << 16) & RED_MASK;
        g32 = (((u32) g) <<  8) & GREEN_MASK;
        b32 = (((u32) b) <<  0) & BLUE_MASK;

        tri_color_b[tri_b_index] = r32 | g32 | b32;


        tri_b_index++;

    } else {
        dx1 = (xy[0][0] - xy[1][0]) / (xy[1][1] - xy[0][1]);                           // x(p1-p2) / y(p2-p1)
        dx2 = (xy[0][0] - xy[2][0]) / (xy[2][1] - xy[0][1]);                           // x(p1-p3) / y(p3-p1)

        // Convert to screen pixel coordinates
        tri_t_dx[tri_t_index][0] = ((s32) (dx1 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;
        tri_t_dx[tri_t_index][0] |= TRI_ACTIVATE_MASK;    // activate this triangle in hardware
        tri_t_dx[tri_t_index][1] = ((s32) (dx2 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;

        for (u32 i=0; i<3; i++) {
            float temp1 = xy[i][0] * vres_q + hres_h + cube_offset;                    // x, shift right to center
            float temp2 = -(xy[i][1] * vres_q) + vres_h;                               // y, different method, scale and subtract
            tri_t_xy[tri_t_index][i][0] = ((s32) (temp1 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;       // x, shift right to center
            tri_t_xy[tri_t_index][i][1] = ((s32) (temp2 * (1 << SLOPE_FIXEDPOINT_FRACTIONAL_BITS))) & SLOPE_MASK;       // y, different method, scale and subtract
        } 

        // Adjust RGB component based on updated intensity
        r = (color_t[tri_t_index] >> 16) & 0xFF;
        g = (color_t[tri_t_index] >>  8) & 0xFF;
        b = (color_t[tri_t_index] >>  0) & 0xFF;

        r = r * intensity;
        g = g * intensity;
        b = b * intensity;

        r32 = (((u32) r) << 16) & RED_MASK;
        g32 = (((u32) g) <<  8) & GREEN_MASK;
        b32 = (((u32) b) <<  0) & BLUE_MASK;

        tri_color_t[tri_t_index] = r32 | g32 | b32;

        tri_t_index++;
    }
}

