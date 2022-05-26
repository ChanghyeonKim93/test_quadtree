#ifndef _MACROS_H_
#define _MACROS_H_

// Distance calculations
#define DIST_EUCLIDEAN(x,y,x_,y_) ((x-x_)*(x-x_)+(y-y_)*(y-y_))
#define DIST_MANHATTAN(x,y,x_,y_) (abs(x-x_)+abs(y-y_))

// Bound check functions
#define INBOUND_RECT(x,y,rect) ((x > rect.tl.x && x < rect.br.x && y > rect.tl.y && y < rect.br.y))

// related to the quadtree node ID
// child idx (flag_we, flag_ns)
// |-------|-------|
// |0 (0,0)|2 (1,0)|
// |-------|-------|
// |1 (0,1)|3 (1,1)|
// |-------|-------|
//
// |  1  |  2  |  3  |  4  |  5  |  ...
// | root| tl0 | bl0 | tr0 | br0 |  ...
// (Z-order)
// 
//  id_node * 4  - 2 -> first child id
// (id_node + 2) / 4 -> parent id
// (id_node + 2) % 4 -> my quadrant index (0~3))

#define BITMASK_EAST  0b10
#define BITMASK_SOUTH 0b01

#define GET_PARENT_ID(id_node) ((id_node+2)>>2)
#define GET_FIRST_CHILD_ID(id_node) ((id_node << 2) - 2)
#define GET_CHILD_ID_FLAGS(id_node,flag_we,flag_ns) ((id_node << 2) - 2 + (flag_we << 1) + flag_ns)
#define GET_CHILD_ID_INDEX(id_node,idx_child) ((id_node << 2) - 2 + idx_child)
#define GET_MY_QUADRANT(id_node) ((id_node+2) % 4)

#define FIND_QUADRANT(x_,y_,qrect,flag_we,flag_ns) {flag_we = x_ > ((qrect.tl.x+qrect.br.x)>>1); flag_ns = y_ > ((qrect.tl.y+qrect.br.y)>>1);}

#endif