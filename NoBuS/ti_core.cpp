// OFDMA on DL and UL in batches of 4 or 2 or 1 with highest-first Tx.

#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <algorithm>    // std::min
#include <math.h>       /* ceil */
#include "ti_core.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <fstream>
#include <iterator>
#include <queue>

#include <map>
#include <string>
/* run this program using the console pauser or add your own getch, system("pause") or input loop */

#define CW_MIN_HA                   3 //3
#define CW_MAX_HA                   7 // 7 AC_HA
#define AIFS_HA                     34 // Assuming AIFSN=1 for AC_HA
#define SIFS                     16
#define SLOT_TIME                9 // 9
#define MAX_PPDU_DURATION_US     5484 //5484µs
#define ETHERNET_FRAME_LENGTH    (2000*8) //length in bits
#define MAX_SIMULATION_TIME_US   20000000 //20 sec
#define PACKET_SIZE_HA				 2500


#define RETRY_COUNT_HA				 8 //4
#define HA_DURATION					1000 // us

using namespace std;


int tone, tone_dl, tone_ul;
int duration_tf = 44, duration_multi_sta_back = 44, duration_back = 44, bsrptime = 44, bsrtime = 44;
int ul_ofdma_ampdu_len=0;

// queue array for storing STA generated packets

std::ofstream global_file;


int bw2ru_size(int bw) {
	//convert bw to ru_size;
	int ru_size = 0;

	if (bw == BANDWIDTH_20MHz) ru_size = RU_SIZE_242_TONES;
	else if (bw == BANDWIDTH_40MHz) ru_size = RU_SIZE_484_TONES;
	else if (bw == BANDWIDTH_80MHz) ru_size = RU_SIZE_996_TONES;
	else if (bw == BANDWIDTH_160MHz) ru_size = RU_SIZE_2x996_TONES;
	else if (bw == BANDWIDTH_320MHz) ru_size = RU_SIZE_4x996_TONES;

	return ru_size;
}

//return the maximum supported Ressource Units (RU) as a function of the channel width
//return 0 if either the channel width or the RU size is unsupported
int getMaxRUsPerChannelWidth(int bw, int ru_size) {
	int maxRUs = 0;
	if (bw == BANDWIDTH_20MHz) {
		if (ru_size == RU_SIZE_26_TONES) maxRUs = 9;
		else if (ru_size == RU_SIZE_52_TONES) maxRUs = 4;
		else if (ru_size == RU_SIZE_106_TONES) maxRUs = 2;
		else if (ru_size == RU_SIZE_242_TONES) maxRUs = 1;
		//else unsupported ru_size
	}
	else if (bw == BANDWIDTH_40MHz) {
		if (ru_size == RU_SIZE_26_TONES) maxRUs = 18;
		else if (ru_size == RU_SIZE_52_TONES) maxRUs = 8;
		else if (ru_size == RU_SIZE_106_TONES) maxRUs = 4;
		else if (ru_size == RU_SIZE_242_TONES) maxRUs = 2;
		else if (ru_size == RU_SIZE_484_TONES) maxRUs = 1;
		//else unsupported ru_size
	}
	else if (bw == BANDWIDTH_80MHz) {
		if (ru_size == RU_SIZE_26_TONES) maxRUs = 37;
		else if (ru_size == RU_SIZE_52_TONES) maxRUs = 16;
		else if (ru_size == RU_SIZE_106_TONES) maxRUs = 8;
		else if (ru_size == RU_SIZE_242_TONES) maxRUs = 4;
		else if (ru_size == RU_SIZE_484_TONES) maxRUs = 2;
		else if (ru_size == RU_SIZE_996_TONES) maxRUs = 1;
		//else unsupported ru_size
	}
	else if (bw == BANDWIDTH_160MHz) {
		if (ru_size == RU_SIZE_26_TONES) maxRUs = 74;
		else if (ru_size == RU_SIZE_52_TONES) maxRUs = 32;
		else if (ru_size == RU_SIZE_106_TONES) maxRUs = 16;
		else if (ru_size == RU_SIZE_242_TONES) maxRUs = 8;
		else if (ru_size == RU_SIZE_484_TONES) maxRUs = 4;
		else if (ru_size == RU_SIZE_996_TONES) maxRUs = 2;
		else if (ru_size == RU_SIZE_2x996_TONES) maxRUs = 1;
		//else unsupported ru_size
	}

	else if (bw == BANDWIDTH_320MHz) {
		if (ru_size == RU_SIZE_26_TONES) maxRUs = 148;
		else if (ru_size == RU_SIZE_52_TONES) maxRUs = 64;
		else if (ru_size == RU_SIZE_106_TONES) maxRUs = 32;
		else if (ru_size == RU_SIZE_242_TONES) maxRUs = 16;
		else if (ru_size == RU_SIZE_484_TONES) maxRUs = 8;
		else if (ru_size == RU_SIZE_996_TONES) maxRUs = 4;
		else if (ru_size == RU_SIZE_2x996_TONES) maxRUs = 2;
		else if (ru_size == RU_SIZE_4x996_TONES) maxRUs = 1;
			//else unsupported ru_size
		}

	return maxRUs;
}

//returns the maximum number of MPDUs within an A-MPDU
int getOfdmaAMpduLength(int mcs, int ru_size, int max_ppdu_duration, int mpdulen) {

	// For 40MHz channel
	double data_rate;
	int cnt = 0;


	if (mcs==9){
		if (ru_size == RU_SIZE_26_TONES) {
			data_rate = 11.8;
		}
		else if (ru_size == RU_SIZE_52_TONES) {
			data_rate = 23.5;
		}
		else if (ru_size == RU_SIZE_106_TONES) {
			data_rate = 50.0;
		}
		else if (ru_size == RU_SIZE_242_TONES) {
			data_rate = 114.7;
		}
		else if (ru_size == RU_SIZE_484_TONES) {
			data_rate = 229.4;
		}
		else if (ru_size == RU_SIZE_996_TONES) {
			data_rate = 480.4;
		}
		else if (ru_size == RU_SIZE_2x996_TONES) {
			data_rate = 960.8;
		}
		else if (ru_size == RU_SIZE_4x996_TONES) {
			data_rate = 1921.6;
		}
	}

//	if (ru_size == RU_SIZE_26_TONES) {
//		data_rate = 8*11.8;
//	}
//	else if (ru_size == RU_SIZE_52_TONES) {
//		data_rate = 8*23.5;
//	}
//	else if (ru_size == RU_SIZE_106_TONES) {
//		data_rate = 8*50.0;
//	}
//	else if (ru_size == RU_SIZE_242_TONES) {
//		data_rate = 8*114.7;
//	}
//	else if (ru_size == RU_SIZE_484_TONES) {
//		data_rate = 8*229.4;
//	}
//	else if (ru_size == RU_SIZE_996_TONES) {
//		data_rate = 480.4;
//	}
//	else if (ru_size == RU_SIZE_2x996_TONES) {
//		data_rate = 960.8;
//	}


	else {
		printf("MCS_%d is not supported !\n", mcs);
		exit(0);
	}

	cnt = data_rate * (max_ppdu_duration - 40) / ((44 + mpdulen)*8); // 40 is the PHY header, and 44 is the MAC header + MPDU delimiter
	return cnt;
}

int getEdcaAMpduLength(int mcs, int bw, int max_ppdu_duration, int mpdulen) {
	int ru_size = bw2ru_size(bw);
	return getOfdmaAMpduLength(mcs, ru_size, max_ppdu_duration, mpdulen);
}

int getPpduDuration(int mcs, int ru_size, int psduLen) {
	int ppdu_duration = 0;
	int Ndbps = 0;
	if ((mcs != 6) && (mcs != 9) && (mcs != 10)) {
		printf("MCS %d is not supported\n", mcs);
		exit(0);
	}

//	Ndbps for 320MHz channel; Ndbps=Ncbps*Nsd*R*Nss; Nss=1
	if (mcs==9){
//		26 tones-->Nsd=24, 52 tones-->Nsd=48, 106 tones-->Nsd=102
//		R=5/6 for mcs=9
		if (ru_size == RU_SIZE_26_TONES) {
			Ndbps = 160;
		}
		else if (ru_size == RU_SIZE_52_TONES) {
			Ndbps = 320; //RU-52, MCS6, 1SS (corresponds to a data rate of 15.9Mbps)
		}
		else if (ru_size == RU_SIZE_106_TONES) {
			Ndbps = 680;
		}
		else if (ru_size == RU_SIZE_242_TONES) {
			Ndbps = 1560;
		}
		else if (ru_size == RU_SIZE_484_TONES) {
			Ndbps = 3120;
		}
		else if (ru_size == RU_SIZE_996_TONES) {
			Ndbps = 6240;
		}
		else if (ru_size == RU_SIZE_2x996_TONES) {
			Ndbps = 12480;
		}
		else if (ru_size == RU_SIZE_4x996_TONES) {
			Ndbps = 24960;
		}
	}


	ppdu_duration = 40 + ceil((16 + psduLen + 6.0)/Ndbps) * (12.8 + 0.8);
	return ppdu_duration;
}

int getOfdmaAMpduDuration(int mcs, int ru_size, int ampdu_len, int mpdulen) {
	int psduLen = (44 + mpdulen) * 8 * ampdu_len;
	return getPpduDuration(mcs, ru_size, psduLen);
}

int getEdcaAMpduDuration(int mcs, int bw, int ampdu_len, int mpdulen) {
	int ru_size = bw2ru_size(bw);
	// changing this to properly convert mpdulen in bytes into bits
	int psduLen = (44 + mpdulen) * 8 * ampdu_len; // (44 * 8 + mpdulen) * ampdu_len;
	return getPpduDuration(mcs, ru_size, psduLen);
}


struct device_AP {
	//variables for RA STAs for channel contention
	int bt_ha; //backoff time HA
	int cw_ha; //contention window HA
	int bt_vi; //backoff time VI
	int cw_vi; //contention window VI

	//variables for RA STAs for RA RU contention
	int obo; //OFDMA BackOff
	int ocw; //OFDMA contention window
	int usedRARU; //to determine if this station used a RA RU that experiences a collision

	//variables used for both channel and RA RU contentions
	long long int lastSent; //the dequeue time of an A-MPDU
	int nSuccAccess_HA, nSuccAccess_VI; //this is not the number of MDPUs but the number of successfully transmitted A-MPDUs (over both the channel and the RUs)
	long long int delay;
	long long int queueheadTime; // this is the timestamp in us of the packet at queue head; different from dequeueTime when node cannot flush the buffer
								 // fully in which case queueheadTime<dequeueTime.
	// variables for EDCA params
	int edca_ampdu_len;
	int edca_ampdu_duration;

	// variables for OFDMA params
	int ofdma_ampdu_len;
	int ofdma_ampdu_duration;
	int sampleCount; // # of samples in buffer
	int nTx; // number of EDCA MPDUs transmitted
	bool incVidCount;
	unsigned int queuesize, max_duration, num_dl_STAs, current_duration;
	vector<bool> dl_stas;
	vector<long long int> last_packet_ha, last_packet_vi, current_size;
	vector<int> dl_stas_ru;
	vector<int> accessTimeVec_ha, accessTimeVec_vi;
	long long int accessTime_ha, accessTime_vi;
	int max_bt_ha, max_bt_vi, index_ap, generated_ha=0, generated_vi=0, dropped_ha=0, dropped_vi=0, count_remove_drop=0, count_remove_sent=0,
			dropped_nobus=0, countSent=0, numAccess=0, numCollisions=0;

	struct delays{
		vector<int> HA;
		vector<int> VI;
	};
	delays delaysList;
};



struct device_sta {
	//variables for RA STAs for channel contention
	int bt_ha; //backoff time HA
	int cw_ha; //contention window HA
	int bt_vi; //backoff time VI
	int cw_vi; //contention window VI


	//variables used for both channel and RA RU contentions
	long long int lastSent_ha, lastSent_vi; //the dequeue time of an A-MPDU
	int nSuccAccess_HA, nSuccAccess_VI, nSuccAccess_mu_HA, nSuccAccess_mu_VI, nSuccAccess_su_HA, nSuccAccess_su_VI; //this is not the number of MDPUs but the number of successfully transmitted A-MPDUs (over both the channel and the RUs)
	long long int delay;
	long long int queueheadTime; // this is the timestamp in us of the packet at queue head; different from dequeueTime when node cannot flush the buffer
								 // fully in which case queueheadTime<dequeueTime.
	// variables for EDCA params
	int edca_ampdu_len;
	int edca_ampdu_duration;

	// variables for OFDMA params
	int ofdma_ampdu_len;
	int ofdma_ampdu_duration;
	int sampleCount; // # of samples in buffer
	int nTx; // number of EDCA MPDUs transmitted
	bool incVidCount;
	struct delays{
		vector<int> HA;
		vector<int> VI;
	};
	delays delaysList, delays_su, delays_mu;
	vector<int> accessTimeVec_ha, accessTimeVec_vi;
	long long int accessTime_ha, accessTime_vi;
	int max_bt_ha, max_bt_vi, count=0, generated_ha=0, generated_vi=0, dropped_ha=0, dropped_nobus=0, dropped_vi=0, count_remove_drop=0, count_remove_sent=0;

};

struct stats_struct {
	int nCollisions; //collisions on the different RA RUs
	int nNoCollisions; //successful transmissions on RA RUs
	int nRATx; //records the number of transmitted MPDUs on RA RUs. The total transmitted MPDUs = nRATx + nSATx
	int nSATx; //records the number of transmitted MPDUs on SA RUs. The total transmitted MPDUs = nRATx + nSATx
};

struct dropped{
	int ha;
	int vi;
};

stats_struct ofdma_stats;

device_AP APSta;
device_sta *RAStas = NULL; //it is possible that nRAstas=0
dropped AC_packets, local_packets;

long long int UL_Tx_count = 0, AP_Tx_count=0; // # UL packet tx., # DL packet tx.
long long int dl_ofdma_tot_dur=0, ul_ofdma_tot_dur=0, dl_ofdma_tot_frame=0, ul_ofdma_tot_frame=0;
vector<int> delay_isolated;
string media, apMedia;
//int maxRUs = getMaxRUsPerChannelWidth(bw, ru_size);
int nSARUs, nDLSTAs, nULSTAs; //this is the number of SA RUs and also the number of SA STAs
int nSAStas, ul_ofdma_count=0, nStas_dl, nStas_ul;
int nSenders_ha = 0, nSenders_vi = 0;
int queue_size, max_duration, max_duration_STA, * access_ap;
vector<vector<int>> delays;
long long int ul_zero_ofdma_ampdu=0, ul_zero_ofdma_access=0, ul_zero_edca_access=0, ul_zero_edca_ampdu=0,
		dl_zero_ofdma_ampdu=0, dl_zero_ofdma_access=0, ampdu_sta_su_VI=0, ampdu_sta_su_HA=0, *ampdu_len, ampdu_sta_mu_VI=0,
		ampdu_sta_mu_HA=0, ampdu_ap_HA=0, ampdu_ap_VI=0;

vector<bool> dl_stas;
vector<int> ampdu_DL_currentsize, ampdu_DL_currentduration, ul_zero_ofdma_delay, ul_zero_edca_delay, dl_zero_ofdma_delay;
unsigned int last_packet_ha, last_packet_vi, buffer_size_ap=0, buffer_size_sta=0, buffer_size_sta_count=0, buffer_size_ap_count=0;
long long int sta_maxbt=0,ap_maxbt=0, *lastSent_temp;
std::multimap<int,int> ul_sampleCount, dl_sampleCount;
int timeleft, dl_ofdma_batch, ul_ofdma_batch, *accesstemp, *accesstemp_su, *accesstemp_mu;
std::map<char,int>::reverse_iterator rit;
vector<int> stas_dl, stas_ul;
bool dlofdma_interrupted;
vector<int> *delaystemp, *delays_mu, *delays_su;


vector<vector<unsigned int>> pacQ_STA_ha, pacQ_AP_ha, retry_cnt_STA_ha, retxQ_STA_ha, retry_cnt_AP_ha, retxQ_AP_ha;
vector<vector<unsigned int>> pacQ_STA_vi, pacQ_AP_vi, retry_cnt_STA_vi, retxQ_STA_vi, retry_cnt_AP_vi, retxQ_AP_vi;
vector<vector<unsigned int>> * pacQ_holder, *retxQ_holder, *retry_cnt_holder;

int nCollisions = 0, nNoCollisions = 0, packet_size, duration, retryCount;
int nTx = 0; //this is the number of transmitted MPDUs using EDCA (UL OFDMA transmissions are excluded and are recorded using other variables)
std::ofstream staO_file("./staO_file.txt"), ap_packet_file("./ap_packet_file.txt");
string device;

dropped updateRetryInfo(int n, int len, int retryCount_local, bool StaDevice, long long int time_now){

	int count_remove_drop=0;
	local_packets.ha = 0;
	device = StaDevice ? "STA ": "AP ";

	// if empty, push new entries
	if ((*retry_cnt_holder)[n].empty()){
		(*retry_cnt_holder)[n].push_back(1);
		(*retxQ_holder)[n].push_back((*pacQ_holder)[n][len]);

			for (int x=0; x<(*retry_cnt_holder)[n].size(); x++){
				staO_file << (*retxQ_holder)[n][x] << "   " << (*retry_cnt_holder)[n][x] << endl;
			}
	}

	else {
		// update existing entries
		if (((*pacQ_holder)[n][len]>=(*retxQ_holder)[n].back())){
			for (int k=0;k<(*retxQ_holder)[n].size();k++){
				(*retry_cnt_holder)[n][k] += 1;
			}

		}
		// add new entries
		if (((*pacQ_holder)[n][len]>(*retxQ_holder)[n].back())){
			(*retxQ_holder)[n].push_back((*pacQ_holder)[n][len]);
			(*retry_cnt_holder)[n].push_back(1);

		}


		// delete MPDUs if retry count threshold is exceeded
		if ((*retry_cnt_holder)[n].front()>retryCount_local){
			for(int y=0;y<(*pacQ_holder)[n].size();y++){

				if ((*retxQ_holder)[n].front()>=(*pacQ_holder)[n][y]){
					++count_remove_drop;
//					printf("\n time: %d  Dropping %d packets of AC-%s from %s\n", time_now, count_remove_drop, media.c_str(), device.c_str());
				}
				else{
					break;
				}
			}


			(*pacQ_holder)[n].erase((*pacQ_holder)[n].begin(), (*pacQ_holder)[n].begin()+count_remove_drop);
			(*retry_cnt_holder)[n].erase((*retry_cnt_holder)[n].begin(), (*retry_cnt_holder)[n].begin()+1);
			(*retxQ_holder)[n].erase((*retxQ_holder)[n].begin(), (*retxQ_holder)[n].begin()+1);
			if ((media.compare("HA")==0)){
				local_packets.ha =count_remove_drop;
			}


		}
	}

	return local_packets;
}

struct wlan_result simulate_wlan(const int bw, int nRAStas, int mcs) {
    //first, check params and print a summary

	int duration_tf = 44, duration_multi_sta_back = 44, duration_back = 44, numaccess=0, bsrptime = 44, bsrtime = 44;
	global_file.open ("global.txt", std::ofstream::out | std::ofstream::app);

	//-------------------------- START OF PACKAGE 1 --------------------------//
	if (nRAStas < 0) nRAStas = 0;
	if (mcs < 0) mcs = 0;
	if (mcs > 11) mcs = 11;



#if 0
    printf("Channel Width                     : %d MHz\n", bw);
    printf("RU size                           : %d tones\n", ru_size);
    printf("Max number of RUs                 : %d\n", maxRUs);
    printf("Number of SA RUs (and SA STAs)    : %d\n", nSARUs);
    printf("Number of RA RUs                  : %d\n", nRARUs);
    printf("Number of STAs (Random Access)    : %d\n", nRAStas);
    printf("MCS index                         : %d\n", mcs);
#endif

	/* initialize random seed: */
    srand (time(NULL)); //put it here because there is a variable called "time", to avoid the error : 'time' cannot be used as a function

    long long int time = 0, last_time=0; //time in µs


	for(int i=0;i<nRAStas;i++){
		APSta.last_packet_ha.push_back(0);
		APSta.last_packet_vi.push_back(0);
	}
	APSta.last_packet_ha.resize(nRAStas);
	APSta.last_packet_vi.resize(nRAStas);


	std::ofstream output_file_AP("./APdelays.txt"), delay_file_STA_VI("./STAdelaysCollect_VI.txt"),
			delay_file_STA_HA("./STAdelaysCollect_HA.txt"), delay_file_AP_HA("./APdelaysCollect_HA.txt"),
			delay_file_AP_VI("./APdelaysCollect_VI.txt");
	std::ofstream output_file_STA("./STA-AMPDU.txt"), output_file_isolated("./delay-DL-isolated.txt");
	std::ofstream output_file_buffer("./buffer.txt");
	std::ofstream txtime_file_sta("./txtime_sta.txt"), txtime_file_ap("./txtime_ap.txt");



	bool waitAifs = true, apCollision = false;

	if (nRAStas > 0) {
	    RAStas = new device_sta[nRAStas];
	}


	//set BT and OBO for all RAStas; only initialization irrespective of channel access method (pure EDCA, pure OFDMA, default OFDMA)
	// bt and cw are for EDCA; obo and ocw for OFDMA
	for (int i = 0; i < nRAStas; i++) {
		RAStas[i].bt_ha = rand() % (CW_MIN_HA + 1);
		RAStas[i].cw_ha = CW_MIN_HA;

		//printf ("%d  ", RAStas[i].bt);


		// dequeuetime is used to measure the channel delays
		//RAStas[i].dequeueTime = 0; //under high load condition, the first A-MPDU is dequeued at t=0
		RAStas[i].nSuccAccess_HA = 0;
		RAStas[i].nSuccAccess_mu_HA = 0;
		RAStas[i].nSuccAccess_su_HA = 0;
		RAStas[i].queueheadTime = 0;
		RAStas[i].delaysList.HA.clear();
		RAStas[i].delays_mu.HA.clear();
		RAStas[i].delays_su.HA.clear();
		RAStas[i].accessTime_ha=0;
		RAStas[i].incVidCount=false;
	}

	APSta.bt_ha = rand() % (CW_MIN_HA + 1);
	APSta.cw_ha = CW_MIN_HA;
	APSta.queueheadTime=0;
	APSta.sampleCount=0;
	APSta.nTx=0;
	APSta.nSuccAccess_HA=0;
	APSta.generated_ha=0;
	APSta.delaysList.HA.clear();
	APSta.incVidCount=false;
	APSta.accessTime_ha = 0;

	delay_isolated.clear();
	stas_dl.clear();


	vector<int> txtime_ap, txtime_sta;
	for (int i=0;i<nRAStas;i++){
		pacQ_STA_ha.push_back(vector<unsigned int>());
		pacQ_STA_ha[i].push_back(0);
		pacQ_AP_ha.push_back(vector<unsigned int>());
		pacQ_AP_ha[i].push_back(0);
		retry_cnt_STA_ha.push_back(vector<unsigned int>());
		retxQ_STA_ha.push_back(vector<unsigned int>());
		retry_cnt_AP_ha.push_back(vector<unsigned int>());
		retxQ_AP_ha.push_back(vector<unsigned int>());

	}

	txtime_ap.clear();
	txtime_sta.clear();

	long long int sum_dur=0, numCollisions=0;
	bool ap_tosend, APhasData;
	vector<int> numPackSent_AP (nRAStas);


	//-------------------------- END OF PACKAGE 1 --------------------------//

	//-------------------------- START OF PACKAGE 2 --------------------------//
    // Generating new HV packets & setting the CW of AP/STAs

	while (time < MAX_SIMULATION_TIME_US) {

		if (time<0)
			exit(0);
		//printf(" -------------- time: %d --------------\n", time);

		ap_tosend=false;
		APSta.incVidCount=false;
		APhasData = false;

		// STA packet generation
		for (int i=0; i<nRAStas; i++){
			// haptic packet generation
			if (pacQ_STA_ha[i].empty()){
				last_packet_ha = RAStas[i].lastSent_ha;
			}
			else{
				last_packet_ha = pacQ_STA_ha[i].back();
			}

			if ((time-last_packet_ha)>=HA_DURATION){

				for (int j=1; j<=floor((time-last_packet_ha)*1.0/HA_DURATION); j++){
					pacQ_STA_ha[i].push_back(last_packet_ha+HA_DURATION*j);
					++RAStas[i].generated_ha;
				}
			}


			// Measuring avg. buffer size only for STA 1
			if (i==1){
				buffer_size_sta += pacQ_STA_ha[i].size();
				++buffer_size_sta_count;
			}
		}

		// AP packet generation
		for (int i=0; i<nRAStas; i++){
			// haptic packet generation
			if (pacQ_AP_ha[i].empty()){
				last_packet_ha = APSta.last_packet_ha[i];
			}
			else{
				last_packet_ha = pacQ_AP_ha[i].back();
			}

			if ((time-last_packet_ha)>=HA_DURATION){

				for (int j=1; j<=floor((time-last_packet_ha)*1.0/HA_DURATION); j++){
					pacQ_AP_ha[i].push_back(last_packet_ha+HA_DURATION*j);
					++APSta.generated_ha;
				}
			}

		}

		nSenders_ha = 0;
		nSenders_vi = 0; // resetting CW of AC_VI even though there is a intra-STA collision

//		cout << "****** Backoff timer status ******\n" << "time:  " << time << endl;
		// setting CW and BT for a new sample at the STAs
		for (int i=0; i<nRAStas; i++){
			RAStas[i].incVidCount = false;
			// update STA  bt, cw when (i) a sample is pending in buffer and STA had just made a Tx.
			//						   (ii) packet(s) were dropped due to RLC threshold
			// when above conditions don't hold, then either the STA has nothing to Tx. or it is in the middle of contention
			if (((pacQ_STA_ha[i].size()>0) && (RAStas[i].bt_ha < 0))){
				RAStas[i].cw_ha = CW_MIN_HA;
				RAStas[i].bt_ha = rand() % (CW_MIN_HA + 1);
				//RAStas[i].accessTime=time;
			}


			// STAs (with old or even new packets) done with backoff
			if (RAStas[i].bt_ha == 0) {
				nSenders_ha += 1;
				RAStas[i].bt_ha = -1; //make it invalid so we can set it later
								   //do not set it here, because BT depends on the transmission status (success of failure)
			}

		}

		//check if the AP will send
		int ap_wins = false;

		// AP has new packets after the last Tx.
		for (int i=0;i<pacQ_AP_ha.size();i++){
			if (pacQ_AP_ha[i].size()>0){
				if (APSta.bt_ha < 0){
					APSta.cw_ha = CW_MIN_HA;
					APSta.bt_ha = rand() % (CW_MIN_HA + 1);
					//APSta.accessTime=time;
				}
				if (APSta.bt_ha == 0) {
					ap_wins = true;
					nSenders_ha += 1;
					APSta.bt_ha = -1; //make it invalid so we can set it later
					//do not set it here, because BT depends on the transmission status (success or failure)
				}
				break;
			}
		}


		// BT should be decreased only when channel is detected free for SLOT_TIME
		// Check if there are STA senders. If yes, BT should not be decreased. So DO NOT decrement BT here


		// AP/STA has to wait for AIFS (in case of EDCA) to start transmitting
		if (waitAifs) {
			if (nSenders_ha>0){
				time += AIFS_HA;
			}
			waitAifs = false;
		}


		//-------------------------- END OF PACKAGE 2 --------------------------//

		//-------------------------- START OF PACKAGE 3 --------------------------//

		//printf("STA BT: %d \t AP BT: %d \t # senders: %d  \n", RAStas[0].bt, APSta.bt, nSenders);

		/*** Handling (no) collisions and advancing time*/
		//if no senders, advance time with slottime. otherwise, advance time with the transmission duration
		if (nSenders_ha == 0) {
			time += SLOT_TIME; //do not "continue", we should decrease BT
//			cout << "time2: " << time << endl;
		}

		// either one of HA-VI or both (belonging to same or different device) have won the contention
		else if ((nSenders_ha == 1)) {
			waitAifs = true;
			nNoCollisions += 1;

			// even if AP wins the contention, need to ensure it has haptic data
			// when only one AC has won
			if (nSenders_ha == 1){
				// if AP wins
				if (ap_wins){
					ap_tosend = true;
				}
			}

			// one HA and one/more ACs have won
			else{
				// If HA belong to AP
				for (int y=0; y<nRAStas; y++){
					if (!pacQ_AP_ha[y].empty()){
						APhasData=true;
						break;
					}
				}

				if (APSta.bt_ha<0 && APhasData){
					ap_tosend=true;
				}

			}
			// if AP alone won the contention
			//no collisions, so simulate an AP transmission
			if (ap_tosend){

				// When AP wins, there should be DL data as it contends only when it has data
				// So initiate DL_OFDMA
				++APSta.numAccess;
				APSta.max_duration=0;
				APSta.num_dl_STAs=0;
				APSta.dl_stas.clear();
				APSta.dl_stas_ru.clear();
				dl_stas.clear();
				ampdu_DL_currentsize.clear();
				ampdu_DL_currentduration.clear();

				if (nSenders_ha==1){
					// Find if queue is empty or not for each STA
					for (int s=0;s<pacQ_AP_ha.size();s++){
						APSta.queuesize = pacQ_AP_ha[s].size();
						if(APSta.queuesize>0){
							++APSta.num_dl_STAs;
							APSta.dl_stas.push_back(true);
						}
						else{
							APSta.dl_stas.push_back(false);
						}
					}
					pacQ_holder=&pacQ_AP_ha;
					retxQ_holder = &retxQ_AP_ha;
					retry_cnt_holder = &retry_cnt_AP_ha;
					packet_size = PACKET_SIZE_HA;
					duration = HA_DURATION;
					media = "HA";
					ampdu_len = &ampdu_ap_HA;
					access_ap = &APSta.nSuccAccess_HA;
					APSta.accessTimeVec_ha.push_back(time-APSta.accessTime_ha);

				}


				if (APSta.num_dl_STAs==1){
					tone=RU_SIZE_484_TONES;
				}
				else if(APSta.num_dl_STAs==2){
					tone=RU_SIZE_242_TONES;
				}
				else if(APSta.num_dl_STAs>=3 && APSta.num_dl_STAs<=4){
					tone=RU_SIZE_106_TONES;
				}
				else if(APSta.num_dl_STAs>=5 && APSta.num_dl_STAs<=8){
					tone=RU_SIZE_52_TONES;
				}
				else if(APSta.num_dl_STAs>=9 && APSta.num_dl_STAs<=18){
					tone=RU_SIZE_26_TONES;
				}

				for(int i=0;i<APSta.dl_stas.size();i++){
					if (APSta.dl_stas[i]){
						APSta.dl_stas_ru.push_back(tone);
					}
					else{
						APSta.dl_stas_ru.push_back(0);
					}
				}


				// Assign RUs for each STA; 0 if empty
				//assignRUs_DL(dl_stas, APSta.num_dl_STAs);

//				cout << "AP: CW -- HA: " << APSta.bt_ha << "  VI: " << APSta.bt_vi << endl;
//				printf("\n ----------------- DL OFDMA start -- %s \t time: %lld ----------------- \n", media.c_str(), time);
				timeleft = MAX_PPDU_DURATION_US;

				int optimal_queue_dl, highest_sta_dl, optimal_dur_dl, optimal_len_dl, dl_remaining, count_remove_sent_dl=0;
				dl_sampleCount.clear();

				// pair up the buffer sizes and STAs
				if (nSenders_ha==1){
					for (int i = 0; i < nRAStas; i++) {
						if ((*pacQ_holder)[i].size()>0){
							dl_sampleCount.insert(std::pair<int,int>(1,i)); // (*pacQ_holder)[i].size()
						}
					}
					delaystemp = &APSta.delaysList.HA;
				}


				// for multiple DL batches during a media access
				// for single batch during an OFDMA episode, uncomment dl_ofdma_batch
				while((dl_sampleCount.size()>0) && (timeleft>40)){ // && (dl_ofdma_batch==0)
					//++dl_ofdma_batch;
					//dlofdma_interrupted = false;
					// finding the highest buffer and corresponding STA
					for (auto it=dl_sampleCount.rbegin(); it!=dl_sampleCount.rend(); ++it){
						//cout << it->first << "\t" << it->second << "\n" << endl;
						optimal_queue_dl = it->first;
						highest_sta_dl = it->second;
						break;
					}

					nStas_dl = dl_sampleCount.size();
					if (nStas_dl>=4){
						nDLSTAs = 4;
						dl_remaining = nStas_dl%4;
						tone_dl = RU_SIZE_996_TONES;
					}
					else if (nStas_dl>=2){
						nDLSTAs = 2;
						dl_remaining = nStas_dl%2;
						tone_dl = RU_SIZE_2x996_TONES;
					}
					else if (nStas_dl==1){
						nDLSTAs = 1;
						dl_remaining = 0;
						tone_dl = RU_SIZE_4x996_TONES;
					}


					optimal_len_dl =  std::min(optimal_queue_dl, getOfdmaAMpduLength(mcs, tone_dl, timeleft, packet_size));
					optimal_dur_dl = getOfdmaAMpduDuration (mcs, tone_dl, optimal_len_dl, packet_size);

					timeleft -= optimal_dur_dl;
//					printf("\n Max AMPDU: %d \t max duration: %d\n", optimal_len_dl, optimal_dur_dl);

					time += optimal_dur_dl + SIFS + duration_multi_sta_back;
//					cout << "time3: " << time << endl;

					if (nSenders_ha==1){
						APSta.accessTime_ha = time;
					}

					// counting the total time spent in DL-OFDMA
					dl_ofdma_tot_dur += optimal_dur_dl;
					//cout << "\nTotal DL duration ---- Prev: " << dl_ofdma_tot_frame-optimal_dur_dl << "\t Now: " << dl_ofdma_tot_dur << endl;

					for (auto rit=dl_sampleCount.rbegin(); rit!=dl_sampleCount.rend(); ++rit){

						stas_dl.push_back(rit->second);

						// tracking the last packet sent
						if (nSenders_ha==1){
							lastSent_temp = &APSta.last_packet_ha[rit->second];
						}


						// # queued packets is less than optimal
						if (rit->first <= optimal_len_dl){
//							printf("\n AMPDU: %d \t max duration: %d\n", rit->first, optimal_dur_dl);

							dl_ofdma_tot_frame += rit->first;
							//cout << "\nTotal DL frames ---- Prev: " << dl_ofdma_tot_frame-rit->first << "\t Now: " << dl_ofdma_tot_frame << endl;

							if (rit->second == 0){
								dl_zero_ofdma_ampdu += rit->first;
								++dl_zero_ofdma_access;
							}

							APSta.nTx += rit->first;
							++(*access_ap);
							//++APSta.nSuccAccesses;

							(*lastSent_temp)=(*pacQ_holder)[rit->second].back();
							++APSta.countSent;
							++numPackSent_AP[rit->second];

							// recording delays for HA and VI
							for (int z=0;z<rit->first;z++){ //
								(*delaystemp).push_back(time-SIFS-duration_multi_sta_back-(*pacQ_holder)[rit->second].back());

								dl_zero_ofdma_delay.push_back((*delaystemp).back());
								if (rit->second == 0){
									output_file_isolated << ((*pacQ_holder)[rit->second].back())/1000.0 << "\t" << (*delaystemp).back() << "\t" <<
											((*pacQ_holder)[rit->second].back())/1000.0+(*delaystemp).back()/1000.0 << "\n";

								}
							}

							APSta.dropped_nobus += (*pacQ_holder)[rit->second].size()-1;
							(*pacQ_holder)[rit->second].clear();

							// remove retry details of sent packets
							(*retxQ_holder)[rit->second].clear();
							(*retry_cnt_holder)[rit->second].clear();

							(*ampdu_len) += rit->first;

							--nDLSTAs;
							count_remove_sent_dl=0;
							if (nDLSTAs==0){
								break;
							}
						}

					}

					// remove sent STAs and go back to DL_OFDMA after first batch if time left
					for (auto it=dl_sampleCount.begin(); it!=dl_sampleCount.end(); ++it){
						std::vector<int>::iterator temp;
						temp = find (stas_dl.begin(), stas_dl.end(), it->second);
						if (temp!=stas_dl.end()){
							dl_sampleCount.erase(it);
						}
					}
					stas_dl.clear();
				}


				APSta.dl_stas.clear();
				APSta.dl_stas_ru.clear();
				APSta.current_size.clear();

				ul_sampleCount.clear();
				int optimal_queue_ul, highest_sta_ul, optimal_dur_ul, optimal_len_ul, ul_remaining, count_remove_sent_ul=0;

				// Same AC that was chosen in DL-OFDMA is selected in UL-OFDMA
				if (nSenders_ha==1){
					// pair up the buffer sizes and STAs
					for (int i = 0; i < nRAStas; i++) {
						if (pacQ_STA_ha[i].size()>0){
							ul_sampleCount.insert(std::pair<int,int>(1,i)); // pacQ_STA_ha[i].size()
						}
					}
					pacQ_holder = &pacQ_STA_ha;
					retxQ_holder = &retxQ_STA_ha;
					retry_cnt_holder = &retry_cnt_STA_ha;
					packet_size = PACKET_SIZE_HA;
					duration = HA_DURATION;
					media = "HA";
				}



 				// Trigger MU-UL transmissions if necessary
				if (timeleft > 0){
					while((ul_sampleCount.size()>0) && (timeleft>40)){ // && (ul_ofdma_batch==0)

						// finding the highest buffer and corresponding STA
						for (auto it=ul_sampleCount.rbegin(); it!=ul_sampleCount.rend(); ++it){
//							cout << it->first << "\t" << it->second << "\t" << (*pacQ_holder)[it->second].back()  << "\n" << endl;
							optimal_queue_ul = it->first;
							highest_sta_ul = it->second;
							break;
						}



						nStas_ul = ul_sampleCount.size();
						if (nStas_ul>=4){
							nULSTAs = 4;
							ul_remaining = nStas_ul%4;
							tone_ul = RU_SIZE_996_TONES;
						}
						else if (nStas_ul>=2){
							nULSTAs = 2;
							ul_remaining = nStas_ul%2;
							tone_ul = RU_SIZE_2x996_TONES;
						}
						else if (nStas_ul==1){
							nULSTAs = 1;
							ul_remaining = 0;
							tone_ul = RU_SIZE_4x996_TONES;
						}


						optimal_len_ul =  std::min(optimal_queue_ul, getOfdmaAMpduLength(mcs, tone_ul, timeleft, packet_size));
						optimal_dur_ul = getOfdmaAMpduDuration (mcs, tone_ul, optimal_len_ul, packet_size);

						timeleft -= optimal_dur_ul;
						time += duration_tf + SIFS + optimal_dur_dl + SIFS + duration_multi_sta_back;

						// counting the total time spent in UL-OFDMA
						ul_ofdma_tot_dur += optimal_dur_ul;

						for (auto rit=ul_sampleCount.rbegin(); rit!=ul_sampleCount.rend(); ++rit){


							stas_ul.push_back(rit->second);

							// tracking the last packet sent
							if (nSenders_ha==1){
								lastSent_temp = &RAStas[rit->second].lastSent_ha;
								delaystemp=&RAStas[rit->second].delaysList.HA;
								delays_mu=&RAStas[rit->second].delays_mu.HA;
								delays_su=&RAStas[rit->second].delays_su.HA;
								accesstemp = &RAStas[rit->second].nSuccAccess_HA;
								//accesstemp_su = &RAStas[rit->second].nSuccAccess_su_HA;
								accesstemp_mu = &RAStas[rit->second].nSuccAccess_mu_HA;
								ampdu_len = &ampdu_sta_mu_HA;
								RAStas[rit->second].accessTimeVec_ha.push_back(time-RAStas[rit->second].accessTime_ha);
								RAStas[rit->second].accessTime_ha=time;
							}


							if (rit->first <= optimal_len_ul){
								if (rit->second == 0){
									ul_zero_ofdma_ampdu += rit->first;
									++ul_zero_ofdma_access;
								}

								ul_ofdma_tot_frame += rit->first;
								RAStas[rit->second].nTx += rit->first;

								RAStas[rit->second].delay = time-SIFS-duration_multi_sta_back-(*pacQ_holder)[rit->second].back();
								RAStas[rit->second].dropped_nobus += pacQ_STA_ha[rit->second].size()-1;
								(*lastSent_temp)=(*pacQ_holder)[rit->second].back();

								// recording delays
								for (int z=0;z<rit->first;z++){ //
									(*delaystemp).push_back((RAStas[rit->second].delay-z*duration));
									(*delays_mu).push_back((RAStas[rit->second].delay-z*duration));

									if (rit->second==0){
										output_file_STA << (*pacQ_holder)[rit->second].back()*1.0/duration+z << "\t" <<
												(*pacQ_holder)[rit->second].back()*1.0/duration+z+(*delaystemp).back()/1000.0 << "\tUL-SA\n";
										ul_zero_ofdma_delay.push_back((*delaystemp).back());
									}

								}

								(*pacQ_holder)[rit->second].clear();

								(*ampdu_len) += rit->first;
								++(*accesstemp_mu);
								++(*accesstemp);

								// reset the BT here if UL-OFDMA is achieved
								if (nSenders_ha==1){
									RAStas[rit->second].bt_ha=-1;
								}
								else{
									RAStas[rit->second].bt_vi=-1;
								}

								// remove retry details of sent packets

								(*retxQ_holder)[rit->second].clear();
								(*retry_cnt_holder)[rit->second].clear();

								--nULSTAs;
								count_remove_sent_ul=0;
								if (nULSTAs==0){
									break;
								}
							}


						}

						// remove sent STAs and go back to DL_OFDMA after first batch if time left
						for (auto it=ul_sampleCount.begin(); it!=ul_sampleCount.end(); ++it){
							std::vector<int>::iterator temp;
							temp = find (stas_ul.begin(), stas_ul.end(), it->second);
							if (temp!=stas_ul.end()){
								ul_sampleCount.erase(it);
							}
						}
						stas_ul.clear();
					}

				}

			}

			else {

				//this is a transmission by a contending STA using EDCA
				// Update time and the no. of packets successfully sent; other steps such as storing sent time, CW update done later

				int j;
				if (nSenders_ha==1){
					duration=HA_DURATION;
					packet_size=PACKET_SIZE_HA;
					pacQ_holder = &pacQ_STA_ha;
					retxQ_holder = &retxQ_STA_ha;
					retry_cnt_holder = &retry_cnt_STA_ha;
					retryCount = RETRY_COUNT_HA;
					media = "HA";
					for (int i = 0; i < nRAStas; i++) {
						if ((RAStas[i].bt_ha < 0) && (!(*pacQ_holder)[i].empty())){
							j=i;
							break;
						}
					}
					delaystemp=&RAStas[j].delaysList.HA;
					delays_su=&RAStas[j].delays_su.HA;
					delays_mu=&RAStas[j].delays_mu.HA;
					// tracking the last packet sent
					lastSent_temp = &RAStas[j].lastSent_ha;
					accesstemp = &RAStas[j].nSuccAccess_HA;
					accesstemp_su = &RAStas[j].nSuccAccess_su_HA;
					ampdu_len=&ampdu_sta_su_HA;
					RAStas[j].accessTimeVec_ha.push_back(time-RAStas[j].accessTime_ha);

				}

				// count the # samples in buffer
				RAStas[j].sampleCount = (*pacQ_holder)[j].size();
				if (RAStas[j].sampleCount>0){
					RAStas[j].edca_ampdu_len =  std::min(1, getEdcaAMpduLength(mcs, bw, MAX_PPDU_DURATION_US, packet_size));
					RAStas[j].edca_ampdu_duration = getEdcaAMpduDuration (mcs, bw, RAStas[j].edca_ampdu_len, packet_size);
					(*ampdu_len) += RAStas[j].edca_ampdu_len;

					RAStas[j].nTx += RAStas[j].edca_ampdu_len;

					if (j == 0){
						ul_zero_edca_ampdu += RAStas[j].edca_ampdu_len;
						++ul_zero_edca_access;
					}

					RAStas[j].queueheadTime = (*pacQ_holder)[j].back();
					(*lastSent_temp) = (*pacQ_holder)[j].back();

					(*pacQ_holder)[j].clear();

					// remove retry details of sent packets
					(*retxQ_holder)[j].clear();
					(*retry_cnt_holder)[j].clear();

					RAStas[j].count_remove_sent=0;
					RAStas[j].delay = time+RAStas[j].edca_ampdu_duration-RAStas[j].queueheadTime;


					// record the delays
					for (int z=0;z<RAStas[j].edca_ampdu_len;z++){ //
						(*delaystemp).push_back((RAStas[j].delay-z*duration));
						(*delays_su).push_back((RAStas[j].delay-z*duration));

					}
					RAStas[j].dropped_nobus += RAStas[j].sampleCount-1;

					time += RAStas[j].edca_ampdu_duration + SIFS + duration_back;

					if (nSenders_ha==1){
						RAStas[j].accessTime_ha = time;
					}

					txtime_sta.push_back(RAStas[j].edca_ampdu_duration);

					if (RAStas[j].edca_ampdu_len<0){
						printf("AMPDU neg. value");
						exit(0);
					}

					++(*accesstemp);
					++(*accesstemp_su);

				}
			}
		}


		// case of collisions

		else {
			++numCollisions;
			// if there is at least 2 HA STAs
			if (nSenders_ha>1){

				// adjust the CW of devices with simultaneous VI winning contention

				time += AIFS_HA;

				APhasData = false;
				apCollision = false;
				for (int y=0; y<nRAStas; y++){
					if (!pacQ_AP_ha[y].empty()){
						APhasData=true;
						break;
					}
				}
				if ((APSta.bt_ha<1) && APhasData){
					apCollision = true;
					apMedia = "HA";
				}

				packet_size=PACKET_SIZE_HA;
				pacQ_holder=&pacQ_STA_ha;
				retry_cnt_holder=&retry_cnt_STA_ha;
				retxQ_holder=&retxQ_STA_ha;
				retryCount=RETRY_COUNT_HA;
				media = "HA";
				// adjust CW in case of vitual collisions

			}


			waitAifs = true;
			nCollisions += 1;

			//if the AP is among the transmitter and is using UL OFDMA, it will stop any transmission
			//after sending the TF so the wasted time is always equal to a station transmission duration
			//if the AP is using DL OFDMA or DL MU-MIMO, it must not come here (actually we consider no collisions in these cases)


			// time should be forwarded by the maximum of duration across STAs and AP
			max_duration_STA = 0;

			// STA part

			for (int i = 0; i < nRAStas; i++) {
				if ((RAStas[i].bt_ha < 0) && (!(*pacQ_holder)[i].empty())){
					RAStas[i].sampleCount = (*pacQ_holder)[i].size();
					RAStas[i].edca_ampdu_len =  std::min(1, getEdcaAMpduLength(mcs, bw, MAX_PPDU_DURATION_US, packet_size)); // RAStas[i].sampleCount
					RAStas[i].edca_ampdu_duration = getEdcaAMpduDuration (mcs, bw, RAStas[i].edca_ampdu_len, packet_size);

					// call function to update retry table entries
					AC_packets = updateRetryInfo(i, RAStas[i].edca_ampdu_len-1, retryCount, true, time);
					RAStas[i].dropped_ha += AC_packets.ha;
					RAStas[i].dropped_vi += AC_packets.vi;


					if (RAStas[i].edca_ampdu_duration > max_duration_STA){
						max_duration_STA = RAStas[i].edca_ampdu_duration;
					}
				}
			}

			// AP part
			if (apCollision){
				++APSta.numCollisions;
				APSta.num_dl_STAs=0;
				APSta.max_duration=0;
				APSta.dl_stas.clear();


				packet_size=PACKET_SIZE_HA;
				pacQ_holder=&pacQ_AP_ha;
				retry_cnt_holder=&retry_cnt_AP_ha;
				retxQ_holder=&retxQ_AP_ha;
				retryCount=RETRY_COUNT_HA;
				media = "HA";


				ampdu_DL_currentsize.clear();
				ampdu_DL_currentduration.clear();

				APSta.dl_stas.clear();
				for (int s=0;s<(*pacQ_holder).size();s++){
					APSta.queuesize = (*pacQ_holder)[s].size();
					if(APSta.queuesize>0){
						++APSta.num_dl_STAs;
						APSta.dl_stas.push_back(true);
					}
					else{
						APSta.dl_stas.push_back(false);
					}
				}


				if (APSta.num_dl_STAs>=4){
					tone = RU_SIZE_996_TONES;
				}
				else if (APSta.num_dl_STAs>=2){
					tone = RU_SIZE_2x996_TONES;
				}
				else if (APSta.num_dl_STAs==1){
					tone = RU_SIZE_4x996_TONES;
				}


				APSta.dl_stas_ru.clear();
				for(int i=0;i<APSta.dl_stas.size();i++){
					if (APSta.dl_stas[i]){
						APSta.dl_stas_ru.push_back(tone);
					}
					else{
						APSta.dl_stas_ru.push_back(0);
					}
				}

				APSta.current_size.clear();
				for (int s=0;s<(*pacQ_holder).size();s++){
					APSta.queuesize=(*pacQ_holder)[s].size();
					queue_size = APSta.queuesize;
					APSta.current_size.push_back(std::min(1, getOfdmaAMpduLength(mcs, APSta.dl_stas_ru[s], MAX_PPDU_DURATION_US, packet_size))); // queue_size
					APSta.current_duration = getOfdmaAMpduDuration (mcs, APSta.dl_stas_ru[s], APSta.current_size[s], packet_size);


					if(APSta.current_duration>APSta.max_duration){
						APSta.max_duration=APSta.current_duration;
					}

					// call function to update the retry table entries
					AC_packets = updateRetryInfo(s, APSta.current_size.back()-1, retryCount, false, time);
					APSta.dropped_ha += AC_packets.ha;
					APSta.dropped_vi += AC_packets.vi;

				}

			}


			APSta.num_dl_STAs=0;
			dl_stas.clear();

			max_duration = (max_duration_STA>=APSta.max_duration) ? max_duration_STA : APSta.max_duration;
			time += max_duration + SIFS + duration_back;
			APSta.max_duration=0;

		}

		//-------------------------- END OF PACKAGE 3 --------------------------//


		//-------------------------- START OF PACKAGE 4 --------------------------//

		/*** BT Update step of STAs based on the (no) collisions ***/

		//if there is no transmission, decrease BT of STAs. Otherwise, do not decrease BT but initialize invalid BT
		for (int i = 0; i < nRAStas; i++) {
			if ((nSenders_ha == 0) && (nSenders_vi==0)) {
				if (RAStas[i].bt_ha>0){
					RAStas[i].bt_ha -= 1;
				}

			}


			// collision b/w HA sources
			else if ((nSenders_ha > 1)) {
				//there are more than 1 sender. find them, increase their CW, and set their BT
				if (RAStas[i].bt_ha < 0 && pacQ_STA_ha[i].size()) {
					RAStas[i].cw_ha = std::min((RAStas[i].cw_ha + 1) * 2 - 1, CW_MAX_HA);
					RAStas[i].bt_ha = rand() % (RAStas[i].cw_ha + 1);
					RAStas[i].max_bt_ha += RAStas[i].bt_ha;
				}


			}
		}

		//-------------------------- END OF PACKAGE 4 --------------------------//

		//-------------------------- START OF PACKAGE 5 --------------------------//

		/*** BT Update step of APs based on the (no) collisions ***/

		//if there is no transmission, decrease BT of AP. Otherwise, do not decrease BT but initialize BT if invalid
		//the AP contends for the channel in the following cases: UL_OFDMA_WITH_EDCA, DL_OFDMA_WITH_EDCA, DL_MU_MIMO_WITH_EDCA
		//the AP does not contend for the channel in PURE_EDCA (we consider that only the STAs contends) and in PURE_UL_OFDMA

		if ((nSenders_ha == 0) && (nSenders_vi==0)) {
			if (APSta.bt_ha>0){
				APSta.bt_ha -= 1;
			}

		}


		else if ((nSenders_ha > 1)) {
			//there are more than 1 sender. find them, increase their CW, and set their BT
			if (APSta.bt_ha < 0 && ap_wins) {
				APSta.cw_ha = std::min((APSta.cw_ha + 1) * 2 - 1, CW_MAX_HA);
				APSta.bt_ha = rand() % (APSta.cw_ha + 1);
				APSta.max_bt_ha += APSta.bt_ha;
			}

		}



		//-------------------------- END OF PACKAGE 5 --------------------------//
	}


	//-------------------------- START OF PACKAGE 6 --------------------------//

	struct wlan_result result;

	double collision_rate = 0;
	if ((nCollisions + nNoCollisions) > 0) collision_rate = nCollisions * 100.0 / (nCollisions + nNoCollisions);

#if 0
	//print some results
	printf("EDCA :\n");
	printf("EDCA A-MPDU length : %d\n", edca_ampdu_len);
	printf("EDCA A-MPDU duration : %dus\n", edca_ampdu_duration);
	printf("Number of EDCA MPDUs : %d\n", nTx);
	printf("Number of successful EDCA transmissions (number of A-MPDUs) : %d\n", nNoCollisions);
	printf("Number of EDCA collisions : %d\n", nCollisions);
	printf("EDCA collision rate : %.2f%%\n", collision_rate);
#endif

	//result.edca_ampdu_len = edca_ampdu_len;
	//result.edca_ampdu_duration = edca_ampdu_duration;
	result.edca_n_tx_mpdu = nTx;
	result.edca_nNoCollisions = nNoCollisions;
	result.edca_nCollisions = nCollisions;
	result.edca_collision_rate = collision_rate;



	collision_rate = 0;
	if ((ofdma_stats.nCollisions + ofdma_stats.nNoCollisions) > 0){
		collision_rate = ofdma_stats.nCollisions * 100.0 / (ofdma_stats.nCollisions + ofdma_stats.nNoCollisions);
	}
	printf("%d \t %d \t %d", AP_Tx_count, UL_Tx_count, RAStas[0].nTx);
	double throughput = AP_Tx_count + UL_Tx_count; //WLAN throughput, i.e. EDCA + UL OFDMA throughput
	throughput = throughput * 1000 * 8 / MAX_SIMULATION_TIME_US;

#if 0
	printf("\n\nOFDMA :\n");
	printf("OFDMA A-MPDU length : %d\n", ofdma_ampdu_len);
	printf("Number of SA transmitted MPDUs : %d\n", ofdma_stats.nSATx);
	printf("Number of successful RA transmitted MPDUs : %d\n", ofdma_stats.nRATx);

	printf("Number of collisions on RA RUs    : %d\n", ofdma_stats.nCollisions);
	printf("Number of no collisions on RA RUs : %d\n", ofdma_stats.nNoCollisions);

	printf("Collision rate : %.2f%%\n", collision_rate);

	printf("\n\nWLAN throughput : %.2f Mbps\n", throughput);
#endif

	//result.ofdma_ampdu_len = ofdma_ampdu_len;
	result.ofdma_n_SA_tx_mpdus = ofdma_stats.nSATx;
	result.ofdma_n_RA_tx_mpdus = ofdma_stats.nRATx;
	result.ofdma_nNoCollisions = ofdma_stats.nCollisions;
	result.ofdma_nCollisions = ofdma_stats.nNoCollisions;
	result.ofdma_collision_rate = collision_rate;

	result.throughput = throughput;

	//record the average tx delays
	//average transmission delays of AP is valid only in cases of DL OFDMA and DL MU-MIMO
	result.avgApTxDelaysMicros = 0;

	float delaytotalglobal=0;
	int samplesglobal=0, numaccess_mu=0, numaccess_su=0;
	float sumsqdiff=0, sumsqdiffglo=0;
	float averg;
	vector <int> delays_allSta;
	long int STA_generated_ha = 0, STA_dropped_ha = 0, AP_generated_ha = 0, AP_dropped_ha = 0,
			STA_generated_vi = 0, STA_dropped_vi = 0, AP_generated_vi = 0, AP_dropped_vi = 0;
	ofstream file;
	string temp;


	printf("\n\n *************** STA Haptic packet statistics ***************\n");

	for (int i=0;i<nRAStas;i++){
		int samples = RAStas[i].delaysList.HA.size();
		printf("\n total samples: %d\n",samples);

		float averg = accumulate(RAStas[i].delaysList.HA.begin(), RAStas[i].delaysList.HA.end(), 0.0)*1.0/(samples*1000);
		delaytotalglobal += averg*samples;
		samplesglobal += samples;

		for (int j=0; j<samples; j++){

			sumsqdiff += (RAStas[i].delaysList.HA[j]/1000.0-averg)*(RAStas[i].delaysList.HA[j]/1000.0-averg);
			delay_file_STA_HA << RAStas[i].delaysList.HA[j] << "\n" ;

		}
		numaccess +=  RAStas[i].nSuccAccess_HA;
		numaccess_mu += RAStas[i].nSuccAccess_mu_HA;
		numaccess_su += RAStas[i].nSuccAccess_su_HA;

		printf ("STA %d \t generated %d \t dropped_RTR %d \t dropped_RTR (%) %f\n", i, RAStas[i].generated_ha, RAStas[i].dropped_ha, RAStas[i].dropped_ha*100.0/RAStas[i].generated_ha);
		printf ("STA %d \t generated %d \t dropped_nobus %d \t dropped_nobus (%) %f\n", i, RAStas[i].generated_ha, RAStas[i].dropped_nobus, RAStas[i].dropped_nobus*100.0/RAStas[i].generated_ha);
		copy(RAStas[i].delaysList.HA.begin(), RAStas[i].delaysList.HA.end(), back_inserter(delays_allSta));

		STA_generated_ha += RAStas[i].generated_ha;
		STA_dropped_ha += RAStas[i].dropped_ha+RAStas[i].dropped_nobus;
	}

	averg = accumulate(delays_allSta.begin(), delays_allSta.end(), 0.0)*1.0/(delays_allSta.size()*1000);

	for (int j=0; j<delays_allSta.size(); j++){
		sumsqdiffglo += (delays_allSta[j]/1000.0-averg)*(delays_allSta[j]/1000.0-averg);
	}

	printf("\nGlobal avg delay (ms): %f \t sumdiff %f \t stddev (ms): %f \n", averg,sumsqdiffglo,sqrt(sumsqdiffglo*1.0/delays_allSta.size()));

	printf("\n\n----------------- AMPDU size -----------------\n");
	printf("Global: %lld \t #Access: %d \t avg. : %f", ampdu_sta_mu_HA+ampdu_sta_su_HA, numaccess,
			(ampdu_sta_mu_HA+ampdu_sta_su_HA)*1.0/numaccess);

	global_file << PACKET_SIZE_HA << "\t" << nRAStas << "\t" << averg << "\t" <<  STA_dropped_ha*1.0/STA_generated_ha << "\t" <<
			(ampdu_sta_mu_HA+ampdu_sta_su_HA)*1.0/numaccess << "\t" << numaccess_su*1.0/nRAStas << "\t" << numaccess_mu*1.0/nRAStas << "\t";

	temp = std::string("/home/vineet/GitRepo/HighFive-NoBuS/ResultsFiles/Haptic/STA-") + std::to_string(nRAStas)
			+ "-" + std::to_string(PACKET_SIZE_HA) + std::string(".txt");
	file.open(temp);
    for (const auto &e : delays_allSta)
    	file << e << "\n";

    file.close();

	printf("\n\n----------------- SU-OFDMA -----------------\n");

	delays_allSta.clear();
	for (int i=0;i<nRAStas;i++){
		copy(RAStas[i].delays_su.HA.begin(), RAStas[i].delays_su.HA.end(), back_inserter(delays_allSta));
	}
	averg = accumulate(delays_allSta.begin(), delays_allSta.end(), 0.0)*1.0/(delays_allSta.size()*1000);

	sumsqdiffglo=0;
	for (int j=0; j<delays_allSta.size(); j++){
		sumsqdiffglo += (delays_allSta[j]/1000.0-averg)*(delays_allSta[j]/1000.0-averg);
	}

	printf("\n----- avg delay (ms): %f \t sumdiff %f \t stddev (ms): %f \n", averg,sumsqdiffglo,sqrt(sumsqdiffglo*1.0/delays_allSta.size()));

	printf("STA: Total AMPDU: %lld \t #Access: %d \t Avg. AMPDU length: %f", ampdu_sta_su_HA, numaccess_su,
			(ampdu_sta_su_HA)*1.0/numaccess_su);

	printf("\n\n----------------- MU-OFDMA -----------------\n");

	delays_allSta.clear();
	for (int i=0;i<nRAStas;i++){
		copy(RAStas[i].delays_mu.HA.begin(), RAStas[i].delays_mu.HA.end(), back_inserter(delays_allSta));
	}

	averg = accumulate(delays_allSta.begin(), delays_allSta.end(), 0.0)*1.0/(delays_allSta.size()*1000);
	sumsqdiffglo=0;
	for (int j=0; j<delays_allSta.size(); j++){
		sumsqdiffglo += (delays_allSta[j]/1000.0-averg)*(delays_allSta[j]/1000.0-averg);
	}

	printf("\n ----- avg delay (ms): %f \t sumdiff %f \t stddev (ms): %f \n", averg,sumsqdiffglo,sqrt(sumsqdiffglo*1.0/delays_allSta.size()));

	printf("STA: Total AMPDU: %lld \t #Access: %d \t Avg. AMPDU length: %f", ampdu_sta_mu_HA, numaccess_mu,
				(ampdu_sta_mu_HA)*1.0/numaccess_mu);


	delaytotalglobal=0;
	samplesglobal=0;
	numaccess_mu=0;
	numaccess_su=0;
	sumsqdiff=0;
	sumsqdiffglo=0;
	averg=0;
	delays_allSta.clear();



	printf("\n\n *************** AP Haptic packet statistics ***************\n");

	float APdelavg = accumulate(APSta.delaysList.HA.begin(), APSta.delaysList.HA.end(), 0.0)*1.0/(APSta.delaysList.HA.size()*1000);
	float sumsqdiffap=0;
	for (int j=0; j<APSta.delaysList.HA.size(); j++){
		sumsqdiffap += (APSta.delaysList.HA[j]/1000.0-APdelavg)*(APSta.delaysList.HA[j]/1000.0-APdelavg);
		delay_file_AP_HA << APSta.delaysList.HA[j] << "\n";

	}

	samplesglobal += APSta.delaysList.HA.size();
	delaytotalglobal = (delaytotalglobal + APdelavg*APSta.delaysList.HA.size())*1.0/samplesglobal;

	printf ("\nAP : \t generated %d \t dropped_RTR %d \t dropped_RTR (%) %f\n",  APSta.generated_ha, APSta.dropped_ha, APSta.dropped_ha*100.0/APSta.generated_ha);
	printf ("\nAP : \t generated %d \t dropped_nobus %d \t dropped_nobus (%) %f\n",  APSta.generated_ha, APSta.dropped_nobus, APSta.dropped_nobus*100.0/APSta.generated_ha);


	printf("\n\n-----------------Global Delay -----------------\n");

	printf("\navg delay (ms): %f \t sumdiff %f \t stddev (ms): %f \n", APdelavg,sumsqdiffap,sqrt(sumsqdiffap*1.0/APSta.delaysList.HA.size()));

	printf("Total AMPDU: %lld \t #Access: %d \t Avg. AMPDU length: %f", ampdu_ap_HA, APSta.nSuccAccess_HA, ampdu_ap_HA*1.0/(APSta.nSuccAccess_HA));

	global_file << APdelavg << "\t" <<  (APSta.dropped_ha+APSta.dropped_nobus)*1.0/APSta.generated_ha << "\t" << ampdu_ap_HA*1.0/(APSta.nSuccAccess_HA) << "\t" <<
			APSta.numAccess << "\t" << numCollisions*1.0/(nRAStas+1) << "\n";
	temp = std::string("/home/vineet/GitRepo/HighFive-NoBuS/ResultsFiles/Haptic/AP-") + std::to_string(nRAStas)
		+ "-" + std::to_string(PACKET_SIZE_HA) + std::string(".txt");
	file.open(temp);
	for (const auto &e : APSta.delaysList.HA)
		file << e << "\n";
	file.close();

	printf("\n\n *******************************************\n");



	printf("\n----------------- Average haptic inter-access time -----------------\n");
	for (int i=0;i<nRAStas;i++){

		printf ("STA %d %f\n", i, accumulate(RAStas[i].accessTimeVec_ha.begin(), RAStas[i].accessTimeVec_ha.end(), 0)*1.0/(RAStas[i].accessTimeVec_ha.size()*1000.0));
	}
//
	printf ("\nAP  %f", accumulate(APSta.accessTimeVec_ha.begin(), APSta.accessTimeVec_ha.end(), 0)*1.0/(APSta.accessTimeVec_ha.size()*1000.0));


	cout << "Sent AP per STA packets " << APSta.countSent*1.0/nRAStas << "\t" << APSta.numAccess << endl;

	for (int i=0; i<nRAStas; i++){
		cout << "AP-" << i << "\t" << numPackSent_AP[i] << endl;
	}
	//-------------------------- END OF PACKAGE 6 --------------------------//

	if (nRAStas > 0) {
		delete[] RAStas;
	}

	return result;
}


