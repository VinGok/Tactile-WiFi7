// OFDMA on DL and UL in batches of 4 or 2 or 1 with highest-first Tx.
// Project used for results in High-five 2.0 paper

// RMSE measurement for DL by isolating latency experienced by one particular stream


#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <math.h>       /* ceil */
#include "ti_core.h"


using namespace std;

void TIoperation(){
	struct wlan_result result;

	const int nStas=12;

	result = simulate_wlan(BANDWIDTH_320MHz, nStas, MCS_9);
}


int main(int argc, char** argv) {
	printf("\n");
	freopen("debug.txt","w",stdout);
	TIoperation();

	printf("\n\nEnd of simulation\n");
	return 0;
}
