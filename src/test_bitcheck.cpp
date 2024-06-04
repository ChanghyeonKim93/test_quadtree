#include <bitset>
#include <iostream>
#include <vector>

/*
  bit 1-4: depth (0~16)
  bit 5  : leaf node
  bit 6  : branch node
  bit 7  : reserved
  bit 8  : reserved
*/

#define BIT_LEAF 0b00010000
#define BIT_BRANCH 0b00100000
#define BITS_LEAF_BRANCH 0b00110000

#define RESET_STATE(state) ((state) == 0b00000000)
#define SET_DEPTH(state, d)               \
  {                                       \
    (state) &= ~0b00001111;               \
    (state) |= static_cast<uint8_t>((d)); \
  }
#define GET_DEPTH(state) \
  (static_cast<uint16_t>((state) & 0b00001111))  // lower four bytes
#define PLUS_DEPTH(state) ((state) += 1)
#define MINUS_DEPTH(state) ((state) -= 1)
#define MAKE_LEAF(state)    \
  {                         \
    (state) &= ~BIT_BRANCH; \
    (state) |= BIT_LEAF;    \
  }
#define MAKE_BRANCH(state) \
  {                        \
    (state) &= ~BIT_LEAF;  \
    (state) |= BIT_BRANCH; \
  }
#define IS_ACTIVATED(state) (((state) & BITS_LEAF_BRANCH) > 0)

#define CHECK_N_TH_BIT(state, n) (((state) & (1 << (n))) > 0)

class QuadNode {
 private:
  uint8_t state_;  // 1 byte
};

int main() {
  uint32_t a;
  uint32_t b;
  uint32_t c;

  uint8_t state = 0;
  state = 4;
  // state (1)
  // unordered_map 구조로 구현 해보자.
  //

  MAKE_LEAF(state);
  std::cout << std::bitset<8>(state) << std::endl;
  std::cout << CHECK_N_TH_BIT(state, 4) << std::endl;

  MAKE_BRANCH(state);
  std::cout << std::bitset<8>(state) << std::endl;
  std::cout << CHECK_N_TH_BIT(state, 5) << std::endl;

  IS_ACTIVATED(state);
  std::cout << std::bitset<8>(state) << std::endl;

  std::cout << GET_DEPTH(state) << std::endl;
  SET_DEPTH(state, 9);
  std::cout << GET_DEPTH(state) << std::endl;

  std::cout << (int)GET_DEPTH(PLUS_DEPTH(state)) << std::endl;
  std::cout << (int)GET_DEPTH(MINUS_DEPTH(state)) << std::endl;

  std::cout << "quadnode size: " << sizeof(QuadNode) << std::endl;
  std::cout << "vector size: " << sizeof(std::vector<int>) << std::endl;

  return 0;
}