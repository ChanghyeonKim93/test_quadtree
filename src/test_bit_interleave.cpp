#include <iostream>
#include <bitset>

using namespace std;
// ref : https://stackoverflow.com/questions/39490345/interleave-bits-efficiently

int main() {
    uint8_t size = 4;
    uint8_t xx[size]; // 8 bits
    uint8_t yy[size]; // 8 bits
    uint16_t zz[size]; // interleaved data. (two times more bits)
    
    static const unsigned int B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
    static const unsigned int S[] = {1, 2, 4, 8};

    unsigned int x = 0b1011; // Interleave lower 16 bits of x and y, so the bits of x
    unsigned int y = 0b0011; // are in the even positions and bits from y in the odd;
    unsigned int z; // z gets the resulting 32-bit Morton Number.  
                    // x and y must initially be less than 65536.
    cout << bitset<8>(x) <<"," <<bitset<8>(y) ;

    x = (x | (x << S[3])) & B[3];
    x = (x | (x << S[2])) & B[2];
    x = (x | (x << S[1])) & B[1];
    x = (x | (x << S[0])) & B[0];

    y = (y | (y << S[3])) & B[3];
    y = (y | (y << S[2])) & B[2];
    y = (y | (y << S[1])) & B[1];
    y = (y | (y << S[0])) & B[0];
    z = y | (x << 1);
/*
(z & 0b10101010) : y 추출
(z | 0b01010101) : 
// top    = ((( (z) & 0b10101010) − 1) & 0b10101010) | ( (z) & 0b01010101) // y 에서 1을 빼고, 다시 합쳐줌.
y 추출, -1 후 다시 y로 만들어줌.

// bottom = ((( (z) | 0b01010101) + 1) & 0b10101010) | ( (z) & 0b01010101) // y 에서 1을 더하고, 다시 합쳐줌.
// left   = ((( (z) & 0b01010101) − 1) & 0b01010101) | ( (z) & 0b10101010) // 
// right  = ((( (z) | 0b10101010) + 1) & 0b01010101) | ( (z) & 0b10101010)

*/
    cout << " / " << bitset<8>(z) <<std::endl;

    for(int i = 0; i < size; ++i){
        xx[i] = i; yy[i] = i;
    }

    for(int j = 0; j < size; ++j){
        for(int i = 0; i < size; ++i){
            cout << "x,y:" 
            << bitset<4>(xx[i]) <<" / " << bitset<4>(yy[i]) 
            << " / "; 
        }
    }

    return 0;
}