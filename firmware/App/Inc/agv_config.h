#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H

#define AGV_GRID_ROWS          6U      // WHY: physical map has 6 logical rows
#define AGV_GRID_COLS          6U      // WHY: physical map has 6 logical columns
#define AGV_CELL_SIZE_M        0.254f  // WHY: 152.4 cm / 6 = 25.4 cm
#define AGV_TRACK_WIDTH_M      0.020f  // WHY: printed black track is 2 cm wide
#define AGV_MAX_PATH_NODES     36U     // WHY: maximum unique 6x6 logical nodes

#endif
