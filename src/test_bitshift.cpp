#include <iostream>
#include <memory>

using namespace std;

#define GETFIRSTCHILD(i) ((i) << 2 - 2)
#define GETPARENT(i) (((i) + 2) >> 2)
#define GETINDEX(i) (((i) + 2) & 0b11)
#define GETINDEX_FLAGS(sn, ew) (((sn) << 1) | (ew))

// #define GETDEPTH(i) (((i)+2) & 0b11)
#define DIV4_QUOTIENT(i) ((i) >> 2)
#define DIV4_REMAINDER(i) ((i) & 0b11)

int main() {
  uint32_t a;
  uint32_t b;
  uint32_t c;

  a = 35;

  std::cout << DIV4_REMAINDER(a) << "," << a % 4 << std::endl;
  std::cout << DIV4_QUOTIENT(a) << "," << a / 4 << std::endl;

  return 0;
}