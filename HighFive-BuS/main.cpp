// Haptic and video streams being transmitted from each STA as AC_HA and AC_VI, respectively.
// Both streams generate frames at 1kHz
// OFDMA on DL and UL in batches of 4 or 2 or 1 with highest-first Tx.
// Project used for results in High-five 2.0 paper

// Full OFDMA and short retry count implemented for buffered scheme (BuS)
// RMSE measurement for DL by isolating latency experienced by one particular stream


// Commands for synchronizing with Github
// git add 80211ax-TI-OFDMA-Win
// git commit -m "comment"
// git push
// git status


#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <math.h>       /* ceil */
#include "ti_core.h"


using namespace std;

void TIoperation(){
	struct wlan_result result;

	setApAntennas(1);
	const int nStas=12;

	result = simulate_wlan(BANDWIDTH_40MHz, 10, nStas, MCS_9);
//	printf("%20.2fMbps ", result.throughput);
//	printf("\n");
}


int main(int argc, char** argv) {
	printf("\n");
	freopen("debug.txt","w",stdout);
//	for (int i=1; i<=10; i++){
	TIoperation();
//	}

	printf("\n\nEnd of simulation\n");
	return 0;
}
