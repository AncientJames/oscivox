#define _GNU_SOURCE
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <sched.h>
#include <time.h>
#include <termios.h>
#include <math.h>

#include "rammel.h"
#include "matrix.h"

#define SHOWREFRESH 1

#define REVOLUTION_PRECISION 20
#define ANGLE_PRECISION 10
#define SINCOS_PRECISION 12
#define SCAN_BPC 3


//Pi 2, 3, 4
#define BCM2708_PERI_BASE        0x20000000
#define BCM2709_PERI_BASE        0x3F000000
#define BCM2711_PERI_BASE        0xFE000000

#define BCM_BASE BCM2711_PERI_BASE

#define GPIO_BASE (BCM_BASE + 0x200000)
#define TIMER_CTRL (BCM_BASE + 0x3000)

#define GPFSEL0      0
#define GPFSEL1      1
#define GPFSEL2      2
#define GPFSEL3      3
#define GPFSEL4      4
#define GPFSEL5      5
#define GPSET0       7
#define GPSET1       8
#define GPCLR0       10
#define GPCLR1       11
#define GPLEV0       13
#define GPLEV1       14

#if (BCM_BASE) == (BCM2711_PERI_BASE)
#define GPPUPPDN0 57
#define GPPUPPDN1 58
#define GPPUPPDN2 59
#define GPPUPPDN3 60
#else
#define GPPUD        37
#define GPPUDCLK0    38
#define GPPUDCLK1    39
#endif


#define SPIN_SYNC 1

#define RGB_0_R1 12
#define RGB_0_G1 9
#define RGB_0_B1 6
#define RGB_0_R2 5
#define RGB_0_G2 8
#define RGB_0_B2 7

#define RGB_1_R1 21
#define RGB_1_G1 13
#define RGB_1_B1 20
#define RGB_1_R2 26
#define RGB_1_G2 19
#define RGB_1_B2 16

#define ROW_A 4
#define ROW_B 15
#define ROW_C 18
#define ROW_D 17
#define ROW_E 14

#define RGB_BLANK 11
#define RGB_CLOCK 0
#define RGB_STROBE 3

#define RGB_BITS ((1<<RGB_0_R1)|(1<<RGB_0_G1)|(1<<RGB_0_B1)\
                 |(1<<RGB_0_R2)|(1<<RGB_0_G2)|(1<<RGB_0_B2)\
                 |(1<<RGB_1_R1)|(1<<RGB_1_G1)|(1<<RGB_1_B1)\
                 |(1<<RGB_1_R2)|(1<<RGB_1_G2)|(1<<RGB_1_B2))

#define ROW_BITS ((1<<ROW_A)|(1<<ROW_B)|(1<<ROW_C)|(1<<ROW_D)|(1<<ROW_E))

static volatile uint32_t *gpio_base;
static volatile uint32_t *timer_uS;

static int16_t sincosfixed[1<<ANGLE_PRECISION][2] = {};
static uint16_t spanoffsets[1<<ANGLE_PRECISION][MATRIX_WIDTH] = {};

static inline void busy_wait(uint32_t uS) {
    uint32_t start = *timer_uS;
    while (*timer_uS - start <= uS);
}

static inline void init_pull(int pin, int pud) {
    // pud: 0:off 1:up 2:down
    _Static_assert(BCM_BASE==BCM2711_PERI_BASE, "2711 specific");

    uint32_t bits = gpio_base[GPPUPPDN0 + (pin>>4)];

    int shift = (pin & 0xf) << 1;
    bits &= ~(3 << shift);
    bits |= (pud << shift);

    gpio_base[GPPUPPDN0 + (pin>>4)] = bits;
}

static inline void init_in(int pin) {
    gpio_base[pin / 10] &= ~(7ull << ((pin % 10) * 3));
}
static inline void init_out(int pin) {
    gpio_base[pin / 10] &= ~(7ull << ((pin % 10) * 3));
    gpio_base[pin / 10] |=  (1ull << ((pin % 10) * 3));

    init_pull(pin, 0);
}


static inline void gpio_set_bits(uint32_t bits) {
    gpio_base[GPSET0] = bits;
}
static inline void gpio_clear_bits(uint32_t bits) {
    gpio_base[GPCLR0] = bits;
}
static inline void gpio_set_pin(int pin) {
    gpio_set_bits(1ul << pin);
}
static inline void gpio_clear_pin(int pin) {
    gpio_clear_bits(1ul << pin);
}

static inline uint32_t gpio_get_bits(uint32_t bits) {
    return gpio_base[GPLEV0] & bits;
}
static inline int gpio_get_pin(int pin) {
    return gpio_get_bits(1ul << pin) != 0;
}

static void init_gpio() {
    init_in(SPIN_SYNC);
    init_pull(SPIN_SYNC, 0);

    init_out(RGB_BLANK);
    gpio_set_pin(RGB_BLANK);

    init_out(RGB_CLOCK);
    init_out(RGB_STROBE);

    init_out(RGB_0_R1);
    init_out(RGB_0_G1);
    init_out(RGB_0_B1);

    init_out(RGB_0_R2);
    init_out(RGB_0_G2);
    init_out(RGB_0_B2);

    init_out(ROW_A);
    init_out(ROW_B);
    init_out(ROW_C);
    init_out(ROW_D);
    init_out(ROW_E);
}

static bool map_gpiomem() {
    int memfd;

    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        perror("Can't open /dev/mem (must be root)");
        return NULL;
    }

    gpio_base = (uint32_t*)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, GPIO_BASE);
    
    void* timer_base = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, TIMER_CTRL);
    timer_uS = (uint32_t*)(timer_base ? (uint8_t*)timer_base + 4 : NULL); // just ignore the upper 32 bits
    
    close(memfd);

    if (gpio_base == MAP_FAILED || timer_base == MAP_FAILED) {
        perror("mmap error");
        return false;
    }

    return true;
}

static inline void tiny_wait(uint count) {
    for (uint i = 0; i < count; ++i) {
        gpio_set_bits(0);
    }
}

static int content_fd;
static void* content_map;
static size_t content_size;

static void* map_shared() {
    content_fd = shm_open("/vortex_double_buffer", O_CREAT | O_RDWR, 0666);
    if (content_fd == -1) {
        perror("shm_open");
        return NULL;
    }

    content_size = VOXELS_X*VOXELS_Y*VOXELS_Z * 2 *sizeof(pixel_t);

    if (ftruncate(content_fd, content_size) == -1) {
        perror("ftruncate");
        return NULL;
    }

    content_map = mmap(NULL, content_size, PROT_READ, MAP_SHARED, content_fd, 0);
    if (content_map == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }

    return content_map;
}

static void* map_file(const char* filename) {
    content_fd = open(filename, O_RDONLY);
    if (content_fd == -1) {
        perror("open");
        return NULL;
    }

    struct stat sb;
    if (fstat(content_fd, &sb) == -1) {
        perror("fstat");
        close(content_fd);
        return NULL;
    }
    content_size = sb.st_size;

    content_map = mmap(0, content_size, PROT_READ, MAP_PRIVATE, content_fd, 0);
    if (content_map == MAP_FAILED) {
        perror("mmap");
        close(content_fd);
        return NULL;
    }

    return content_map;
}

static void unmap_content() {
    munmap(content_map, content_size);
    close(content_fd);
}

static bool verbose = false;
static const char* content_filename;

static uint32_t frame_zero = 0;


void parse_options(int argc, char** argv) {
    verbose = false;
    content_filename = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "vz:")) != -1) {
        switch (opt) {
            case 'v':
                verbose = true;
                break;

            case 'z':
                if (optarg) {
                    frame_zero = atoi(optarg);
                }
                break;

        }
    }

    if (optind < argc) {
        content_filename = argv[optind];
    }
}

static uint32_t sync_prev = 0;
static uint32_t revolution_angle = 0;
static uint32_t revolution_period = 1<<26;
static uint32_t revolution_frequency = 1;
static int sync_level = 1;

static uint32_t current_angle() {
    uint32_t tick_curr = *timer_uS;

    int sync = gpio_get_pin(SPIN_SYNC);

    if (sync != sync_level) {
        sync_level = sync;
        if (sync) {
            revolution_period = tick_curr - sync_prev;
            sync_prev = tick_curr;

            revolution_period = max(1<<10, revolution_period);
            revolution_frequency = (1<<REVOLUTION_PRECISION) / revolution_period;
        }
    }

    revolution_angle = (tick_curr - sync_prev) * revolution_frequency;
    return revolution_angle;
}

void init_angles() {
    for (uint i = 0; i < count_of(sincosfixed); ++i) {
        double a = ((double)i * M_PI * 2.0) / (double)(count_of(sincosfixed) - 1);
        double s = sin(a);
        double c = cos(a);
        sincosfixed[i][0] = (int)round(c * (double)(1<<SINCOS_PRECISION));
        sincosfixed[i][1] = (int)round(s * (double)(1<<SINCOS_PRECISION));
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//   F################E          EF 160
//                    D          DE 18.5
//                   / \         .
//                  / . \        .
//                 /  |  \       BD 160
//                /   .   \      .
//               /    |    \     .
//             B/     .     \B`  .
//             |      |      |   AB 30
//             A      C      A`  AC 80
//             
void init_spans() {
    const double scale_horizontal = 0.8;
    const double scale_vertical = 0.8;

    const double ac = 80.0;
    const double ab = 30.0;
    const double bd = 160.0;
    const double de = 18.5;
    const double ef = 160.0;

    const double pitch = ef / (double)MATRIX_WIDTH;

    const double vx = -ef / 2.0;
    const double vy = de + sqrt((bd*bd)-(ac*ac));
    const double vsh = 1.0 / (pitch * scale_horizontal);
    const double vsv = 1.0 / (pitch * scale_vertical);

    for (uint i = 0; i < count_of(spanoffsets); ++i) {
        double theta = ((double)i * M_PI * 2.0) / (double)(count_of(spanoffsets) - 1);
        double bx = cos(theta) * ab - ac;
        double by = sin(theta) * ab;
        
        double ddy = sqrt((bd*bd)-(bx*bx));
        double dy = by + ddy;
        double thbd = atan2(ddy, bx);
        double thef = (M_PI * 2.0 / 3.0) - thbd;

        double px = cos(thef);
        double py = sin(thef);

        double ex = -py * de;
        double ey = dy + px * de;

        for (int l = 0; l < MATRIX_WIDTH; ++l) {
            double el = (0.5 + (double)l) * pitch;

            double lx = ex - px * el;
            double ly = ey - py * el;

            int mx = (((lx - vx) * vsh) + ((double)(VOXELS_X-1) * 0.5)) + 0.5;
            int my = (((ly - vy) * vsv) + ((double)(VOXELS_Z-1) * 0.5)) + 0.5;

            if (mx >= 0 && mx < VOXELS_X && my >= 0 && my < VOXELS_Z) {
                spanoffsets[i][MATRIX_WIDTH-1-l] = mx * VOXEL_X_STRIDE + my * VOXEL_Z_STRIDE;
            } else {
                spanoffsets[i][MATRIX_WIDTH-1-l] = ~0;
            }

        }
    }
}


int main(int argc, char** argv) {
    parse_options(argc, argv);

    cpu_set_t cpu_mask;
    CPU_ZERO(&cpu_mask);
    int target_core = 3;
    CPU_SET(target_core, &cpu_mask);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpu_mask) != 0) {
        perror("sched_setaffinity");
        return 1;
    }

    if (!map_gpiomem()) {
        return -1;
    }

    init_gpio();

    pixel_t* content = NULL;
    if (content_filename) {
        content = map_file(content_filename);
    } else {
        content = (pixel_t*)map_shared();
    }
    if (!content) {
        fprintf(stderr, "Can't open content\n");
        return -1;
    }

    init_angles();
    init_spans();

#if SHOWREFRESH
    uint32_t perf_period = 0;
    uint perf_count = 0;
#endif

    while (!kbhit()) {
#if SHOWREFRESH
        uint32_t frame_start = *timer_uS;
        ++perf_count;
#endif

        for (int c = 0; c < MATRIX_HEIGHT / 2; ++c) {
            uint angle = (current_angle() >> (REVOLUTION_PRECISION - ANGLE_PRECISION)) & (count_of(spanoffsets) - 1);
            uint16_t* span = spanoffsets[angle];

            for (int b = 0; b < SCAN_BPC; ++b) {

                int unblank = MATRIX_WIDTH - (60 >> ((b+(SCAN_BPC-1)) % (SCAN_BPC)));
                for (int r = 0; r < MATRIX_WIDTH; r++) {
                    uint32_t rgbbits = 0;

                    uint offset = span[r];
                    if (offset <= ((VOXELS_X-1)*VOXEL_X_STRIDE + (VOXELS_Z-1)*VOXEL_Z_STRIDE)) {

                        uint vox = c * VOXEL_Y_STRIDE + offset;
                        pixel_t p0 = content[vox];
                        pixel_t p1 = content[vox + VOXEL_FIELD];

                        rgbbits |= (R_MTH_BIT(p0, b) << RGB_0_R1);
                        rgbbits |= (G_MTH_BIT(p0, b) << RGB_0_G1);
                        rgbbits |= (B_MTH_BIT(p0, b) << RGB_0_B1);

                        rgbbits |= (R_MTH_BIT(p1, b) << RGB_0_R2);
                        rgbbits |= (G_MTH_BIT(p1, b) << RGB_0_G2);
                        rgbbits |= (B_MTH_BIT(p1, b) << RGB_0_B2);
                    }

                    gpio_set_bits(rgbbits&RGB_BITS);
                    gpio_clear_bits(((~rgbbits)&RGB_BITS) | ((r==unblank)<<RGB_BLANK));

                    tiny_wait(3);
                    gpio_set_pin(RGB_CLOCK);                // clock it in
                    tiny_wait(3);
                    gpio_clear_pin(RGB_CLOCK);              // clock low
                }

                gpio_set_pin(RGB_BLANK);
                gpio_set_pin(RGB_STROBE);
                tiny_wait(1);

                uint32_t rowbits = ((c & 0x01) << (ROW_A  ))
                                | ((c & 0x02) << (ROW_B-1))
                                | ((c & 0x04) << (ROW_C-2))
                                | ((c & 0x08) << (ROW_D-3))
                                | ((c & 0x10) << (ROW_E-4));
                gpio_clear_bits(((~rowbits) & ROW_BITS));
                gpio_set_bits(rowbits & ROW_BITS);

                gpio_clear_pin(RGB_STROBE);
                tiny_wait(1);
            }
        }

#if SHOWREFRESH
        perf_period += *timer_uS - frame_start;
        if (perf_count >= 1024) {
            printf("%u\n", perf_period / perf_count);
            perf_count = 0;
            perf_period = 0;
        }
#endif
    }

    gpio_set_pin(RGB_BLANK);

    unmap_content();

    return 0;
}


