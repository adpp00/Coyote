#include "tcp_perf_client.hpp"


void status_handler(hls::stream<appTxRsp>& txStatus,
	hls::stream<appTxRsp>&	txStatusBuffer)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	if (!txStatus.empty()){
		appTxRsp resp = txStatus.read();
		txStatusBuffer.write(resp);
	}
}

//Buffers open status coming from the TCP stack
void openStatus_handler(hls::stream<openStatus>& openConStatus,
	hls::stream<openStatus>&	openConStatusBuffer)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	if (!openConStatus.empty())
	{
		openStatus resp = openConStatus.read();
		openConStatusBuffer.write(resp);
	}
}

void txMetaData_handler(hls::stream<appTxMeta>&	txMetaDataBuffer, 
	hls::stream<appTxMeta>& txMetaData)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	if (!txMetaDataBuffer.empty()){
		appTxMeta metaDataReq = txMetaDataBuffer.read();
		txMetaData.write(metaDataReq);
	}
}

template <int WIDTH>
void txDataBuffer_handler(hls::stream<net_axis<WIDTH>>& txDataBuffer,
	hls::stream<ap_axiu<WIDTH, 0, 0, 0>>& txData)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	if (!txDataBuffer.empty()){
		net_axis<WIDTH> in = txDataBuffer.read();
		ap_axiu<WIDTH,0,0,0> out;
		out.data = in.data;
		out.keep = in.keep;
		out.last = in.last;
		txData.write(out);
	}
}



template <int WIDTH>
void inst_client(
				hls::stream<ipTuple>&				openConnection,
            	hls::stream<openStatus>& 			openConStatusBuffer,
				hls::stream<ap_uint<16> >&			closeConnection,
				hls::stream<appTxMeta>&				txMetaDataBuffer,
				hls::stream<net_axis<WIDTH> >& 	txDataBuffer,
				hls::stream<appTxRsp>&	txStatus,
				ap_uint<1>		runTx,
				ap_uint<16>		numSessions,
				ap_uint<8> 		pkgWordCount,
				ap_uint<32>		serverIpAddress,
				ap_uint<8>		TotalPkgPerConn)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

enum iperfFsmStateType {IDLE, INIT_CON, WAIT_CON, CONSTRUCT_HEADER, INIT_RUN, START_PKG, CHECK_REQ, WRITE_PKG, CHECK_SIZE};
static iperfFsmStateType iperfFsmState = IDLE;

static ap_uint<16> currentSessionID = 0;
static ap_uint<16> numConnections   = 0;
static ap_uint<8>  wordCount        = 0;
static ap_uint<16> totalPkgCounter  = 0; 
static ap_uint<16> totalSendTarget  = 0; 


switch (iperfFsmState){
	case IDLE:
		if(runTx){
			currentSessionID = 0;
			numConnections   = 0;
			totalPkgCounter  = 0;
			totalSendTarget  = 0;
			iperfFsmState = INIT_CON;
		}
	break;
	
	case INIT_CON:{
		ipTuple openTuple;
		openTuple.ip_address = serverIpAddress;
		openTuple.ip_port = 5001;
		currentSessionID++;
		openConnection.write(openTuple);
		if(currentSessionID == numSessions){
			currentSessionID = 0;
			iperfFsmState = WAIT_CON;
		}
	}
	break;

	case WAIT_CON:{

		if(currentSessionID == numSessions){
			if(numConnections > 0){
				totalSendTarget = numConnections * TotalPkgPerConn;
				iperfFsmState   = CHECK_REQ;
			}
			else{
				iperfFsmState   = IDLE; 
			}
		}
		else if(!openConStatusBuffer.empty()){
			openStatus status = openConStatusBuffer.read();
			if(status.success){
				txMetaDataBuffer.write(appTxMeta(status.sessionID, pkgWordCount*(WIDTH/8)));
				numConnections++;
			}
			currentSessionID++;
		}
	}
	break;
	case CHECK_REQ:
		if (!txStatus.empty())
		{
			appTxRsp resp = txStatus.read();
			if (resp.error == 0){
				currentSessionID = resp.sessionID;
				iperfFsmState = START_PKG;
			}
			else{	
				if (resp.error == 1){
					numConnections--;
				}
				else{
					txMetaDataBuffer.write(appTxMeta(resp.sessionID, pkgWordCount*(WIDTH/8)));
				}
			}
		}
	break;	
	case START_PKG:{
		if(totalPkgCounter < totalSendTarget - numConnections){
			txMetaDataBuffer.write(appTxMeta(currentSessionID, pkgWordCount*(WIDTH/8)));
		}
		net_axis<WIDTH> currWord;
		for (int i = 0; i < (WIDTH/64); i++)
		{
			#pragma HLS UNROLL
			currWord.data(i*64+63, i*64) = 0x3736353433323130ULL;
			currWord.keep(i*8+7, i*8) = 0xff;
		}
		wordCount = 1;
		currWord.last = (wordCount == pkgWordCount);
		txDataBuffer.write(currWord);
		if (currWord.last)
		{
			wordCount = 0;
			totalPkgCounter ++;
			iperfFsmState = CHECK_SIZE;
		}
		else iperfFsmState = WRITE_PKG;
	}
	break;
	case WRITE_PKG:
	{
		wordCount++;
		net_axis<WIDTH> currWord;
		for (int i = 0; i < (WIDTH/64); i++) 
		{
			#pragma HLS UNROLL
			currWord.data(i*64+63, i*64) = 0x3736353433323130ULL;
			currWord.keep(i*8+7, i*8) = 0xff;
		}
		currWord.last = (wordCount == pkgWordCount);
		txDataBuffer.write(currWord);
		if (currWord.last)
		{
			wordCount = 0;
			totalPkgCounter ++;
			iperfFsmState = CHECK_SIZE;
		}
	}
	break;
	case CHECK_SIZE:
	{
		if (totalPkgCounter >= totalSendTarget - numConnections){
			closeConnection.write(currentSessionID);
			if(--numConnections == 0){
				iperfFsmState = IDLE;
			}
			else{
				iperfFsmState = CHECK_REQ;
			}		
		}
		else{
			iperfFsmState = CHECK_REQ;
		}
	}
	break;
	}
}


void tcp_perf_client(
					hls::stream<ipTuple>& openConnection,
					hls::stream<openStatus>& openConStatus,
					hls::stream<ap_uint<16> >& closeConnection,
					hls::stream<appTxMeta>& txMetaData,
					hls::stream<ap_axiu<DATA_WIDTH, 0, 0, 0> >& txData,
					hls::stream<appTxRsp>& txStatus,
					ap_uint<1>		runTx,
					ap_uint<16>		numSessions,
					ap_uint<8> 		pkgWordCount,
					ap_uint<32>		serverIpAddress,
					ap_uint<8>		TotalPkgPerConn)

{
	#pragma HLS DATAFLOW disable_start_propagation
	#pragma HLS INTERFACE ap_ctrl_none port=return

	#pragma HLS INTERFACE axis register port=openConnection name=m_axis_open_connection
	#pragma HLS INTERFACE axis register port=openConStatus name=s_axis_open_status
	#pragma HLS aggregate compact=bit variable=openConnection
	#pragma HLS aggregate compact=bit variable=openConStatus

	#pragma HLS INTERFACE axis register port=closeConnection name=m_axis_close_connection

	#pragma HLS INTERFACE axis register port=txMetaData name=m_axis_tx_meta
	#pragma HLS INTERFACE axis register port=txData name=m_axis_tx_data
	#pragma HLS INTERFACE axis register port=txStatus name=s_axis_tx_status
	#pragma HLS aggregate compact=bit variable=txMetaData
	#pragma HLS aggregate compact=bit variable=txStatus

	#pragma HLS INTERFACE ap_none register port=runTx
	#pragma HLS INTERFACE ap_none register port=numSessions
	#pragma HLS INTERFACE ap_none register port=pkgWordCount
	#pragma HLS INTERFACE ap_none register port=serverIpAddress
	#pragma HLS INTERFACE ap_none register port=TotalPkgPerConn

	//This is required to buffer up to 128 reponses
	static hls::stream<appTxRsp>	txStatusBuffer("txStatusBuffer");
	#pragma HLS STREAM variable=txStatusBuffer depth=128

	//This is required to buffer up to 128 reponses => supporting up to 128 connections
	static hls::stream<openStatus>	openConStatusBuffer("openConStatusBuffer");
	#pragma HLS STREAM variable=openConStatusBuffer depth=128

	//This is required to buffer up to 128 tx_meta_data => supporting up to 128 connections
	static hls::stream<appTxMeta>	txMetaDataBuffer("txMetaDataBuffer");
	#pragma HLS STREAM variable=txMetaDataBuffer depth=128

	//This is required to buffer up to MAX_SESSIONS txData 
	static hls::stream<net_axis<DATA_WIDTH>>	txDataBuffer("txDataBuffer");
	#pragma HLS STREAM variable=txDataBuffer depth=128

	status_handler(txStatus, txStatusBuffer);
	openStatus_handler(openConStatus, openConStatusBuffer);
	txMetaData_handler(txMetaDataBuffer, txMetaData);
	txDataBuffer_handler<DATA_WIDTH>(txDataBuffer, txData);

	/*
	 * client
	 */
	inst_client<DATA_WIDTH>(	
		openConnection,
		openConStatusBuffer,
		closeConnection,
		txMetaDataBuffer,
		txDataBuffer,
		txStatusBuffer,
		runTx,
		numSessions,
		pkgWordCount,
		serverIpAddress,
		TotalPkgPerConn);

}
