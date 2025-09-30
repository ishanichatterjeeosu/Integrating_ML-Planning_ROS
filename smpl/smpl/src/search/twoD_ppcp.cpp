#include <iostream>

#include <queue>
#include <smpl/search/common.h>
#include <smpl/search/twoD_ppcp.h>
#include <algorithm>

bool debug_ppcp = false;
using namespace std;

void Ppcp::hello()
{
    bel_state_ppcp *X = new bel_state_ppcp();
    X->sx.x = 2;
    X->sx.y = 3;
    X->hx = "uuuu";
    map_bel_ppcp["a"] = X;
    std::cout << "hello................ " << std::endl;
}


