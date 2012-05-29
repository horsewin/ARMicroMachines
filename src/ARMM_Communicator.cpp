#ifndef ARMM_VRPN_CPP
#define ARMM_VRPN_CPP
#define REP(i,n) for(int i=0;i<(int)n;++i)

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <stdio.h>
#include <tchar.h>
#include <math.h>
#include <iostream>

#include "main.h"
#include "ARMM_Communicator.h"
#include "Physics/bt_ARMM_hand.h"

//---------------------------------------------------------------------------
// Global
//---------------------------------------------------------------------------
#ifdef SIM_MICROMACHINE
extern osg::Quat CarsOrientation[NUMBER_CAR];
extern osg::Quat WheelsOrientaion[NUMBER_CAR][NUM_WHEEL];
extern osg::Vec3d CarsPosition[NUMBER_CAR];
extern osg::Vec3d WheelsPosition[NUMBER_CAR][NUM_WHEEL];
extern int input_key;
extern int collide[2];
//extern IplImage *transDepth160;
//extern float *ground_grid;
#endif


//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
const int HANDS_BUFFER = 3000;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
ARMM_Communicator::ARMM_Communicator( vrpn_Connection_IP *c) :
vrpn_Tracker( "ARMM_Comm", c ) 
{
	num_sensors = NUMBER_CAR*NUM_WHEEL+NUMBER_CAR;
	register_types();

	// initialize the hand position
	REP(i,UDP_LIMITATION){
		REP(j,3){
			hand[i][j] = 0;
		}
	}
}

ARMM_Communicator::~ARMM_Communicator()
{
}

//Atsushi
int ARMM_Communicator::register_types(void)
{
	// to handle hand state changes
	hand_m_id = d_connection->register_message_type("vrpn_Tracker Hand");
	return 0;
}

//Atsushi
int	ARMM_Communicator::encode_hand_to(char *buf, int division)
{
	char *bufptr = buf;
	int  buflen =	HANDS_BUFFER;

	// Message includes: long sensor, long scrap, vrpn_float64 pos[3], vrpn_float64 quat[4]
	// Byte order of each needs to be reversed to match network standard

	vrpn_buffer(&bufptr, &buflen, d_sensor);
	//vrpn_buffer(&bufptr, &buflen, d_sensor);

	REP(i,UDP_LIMITATION){
		REP(j,3){
			vrpn_buffer(&bufptr, &buflen, hand[i][j]);
		}
	}

	return HANDS_BUFFER - buflen;
}

void ARMM_Communicator::mainloop() 
{
	//----->Set the number of sending data
	const int CAR_PARAM  = NUMBER_CAR*NUM_WHEEL+NUMBER_CAR;
	const int object_num = static_cast<vrpn_int32>( (*m_objects_body).size() ); 
	int hands_num  = (*m_hands_body).size();
	std::vector<float *> hand_x;
	std::vector<float *> hand_y;
	std::vector<float *> hand_z;
	hand_x.clear();
	hand_y.clear();
	hand_z.clear();

	num_sensors = CAR_PARAM  + 1;
	if(  hands_num > 0){
		REP(i,hands_num){
			hand_x.push_back((*m_hands_body)[i]->debugHandX());
			hand_y.push_back((*m_hands_body)[i]->debugHandY());
			hand_z.push_back((*m_hands_body)[i]->debugHandZ());
		}
	}
	// Update the number of sensors -> (car + 4 wheels) * 2 + key_input + virtual objects
	if( object_num > 0 ){
		num_sensors += object_num;
	}

	//----->Send cars info first
#ifdef SIM_MICROMACHINE
	for(int i =0; i < num_sensors; i++) {
		vrpn_gettimeofday(&_timestamp, NULL);
		vrpn_Tracker::timestamp = _timestamp;
		d_sensor = i;
		if( i < CAR_PARAM){
			// pos and orientation of cars
			if(i % 5 == 0)
			{
				int j = (int) i / 5;
				pos[0] = (vrpn_float64) CarsPosition[j].x(); 
				pos[1] = (vrpn_float64) CarsPosition[j].y(); 
				pos[2] = (vrpn_float64) CarsPosition[j].z(); 
				d_quat[0] = (vrpn_float64) CarsOrientation[j].x();
				d_quat[1] = (vrpn_float64) CarsOrientation[j].y();
				d_quat[2] = (vrpn_float64) CarsOrientation[j].z();
				d_quat[3] = (vrpn_float64) CarsOrientation[j].w();
			} 
			//pos and orientation of car's wheels
			else 
			{ 
				int j = (int) floor((float) i/5);
				int k = (i % 5) -1;
				pos[0] = (vrpn_float64) WheelsPosition[j][k].x(); 
				pos[1] = (vrpn_float64) WheelsPosition[j][k].y(); 
				pos[2] = (vrpn_float64) WheelsPosition[j][k].z();
				d_quat[0] = (vrpn_float64) WheelsOrientaion[j][k].x();
				d_quat[1] = (vrpn_float64) WheelsOrientaion[j][k].y();
				d_quat[2] = (vrpn_float64) WheelsOrientaion[j][k].z();
				d_quat[3] = (vrpn_float64) WheelsOrientaion[j][k].w();
			}
		}		
		//----->Keyboard input checker
		else if( i == CAR_PARAM)
		{
			pos[0] = (vrpn_float64)input_key;

      //TODO �Ƃ肠�����Փ˔�����N���C�A���g�ɂ����邽�߂̏���
      //�����Ŏg��
      pos[1] = (vrpn_float64)collide[0];
      pos[2] = (vrpn_float64)collide[1];
      //std::cout << pos[1] << "," << pos[2] << std::endl;
		}
		//----->Send objects info
		else
		{
			int index = i - (CAR_PARAM  + 1);
			btVector3 trans = (*m_objects_body)[index]->getCenterOfMassTransform().getOrigin();
			btQuaternion quats = (*m_objects_body)[index]->getCenterOfMassTransform().getRotation();
			float bullet_scale_correction = 1;
			pos[0] = static_cast<vrpn_float64>(trans.x()) * bullet_scale_correction;
			pos[1] = static_cast<vrpn_float64>(trans.y()) * bullet_scale_correction;
			pos[2] = static_cast<vrpn_float64>(trans.z()) * bullet_scale_correction;
			d_quat[0] = static_cast<vrpn_float64>(quats.getX()) * bullet_scale_correction;
			d_quat[1] = static_cast<vrpn_float64>(quats.getY()) * bullet_scale_correction;
			d_quat[2] = static_cast<vrpn_float64>(quats.getZ()) * bullet_scale_correction;
			d_quat[3] = static_cast<vrpn_float64>(quats.getW()) * bullet_scale_correction;
		}
		ObjectMessagePacking();
	}

	//----->Send hands info
	if( hands_num > 0){
		d_sensor = 0;
		int hand_pixel = 0;
		REP(index,HAND_SIZE){
			if( hand_z[0][index] < 100 && hand_pixel < UDP_LIMITATION){
				//d_sensor = index;
				hand[hand_pixel][0] = static_cast<vrpn_float32>(hand_x[0][index]); 
				hand[hand_pixel][1] = static_cast<vrpn_float32>(hand_y[0][index]); 
				hand[hand_pixel][2] = static_cast<vrpn_float32>(hand_z[0][index]);
				hand_pixel++;
			}
			if( hand_pixel >= UDP_LIMITATION) break;
		}	
		d_sensor = hand_pixel;

		HandMessagePacking();
	}
	
	//Update server main loop
  server_mainloop();

#endif
}

void ARMM_Communicator::ObjectMessagePacking( void )
{
	char msgbuf[1000];
	int  len = vrpn_Tracker::encode_to(msgbuf);
	if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf, vrpn_CONNECTION_LOW_LATENCY)) {
		fprintf(stderr,"can't write message: tossing\n");
	}
}

void ARMM_Communicator::HandMessagePacking( void )
{
	char msgbuf_h[HANDS_BUFFER];
	int  length = encode_hand_to(msgbuf_h, 1);
	if ( d_connection->pack_message(length, _timestamp, hand_m_id, d_sender_id, msgbuf_h, vrpn_CONNECTION_LOW_LATENCY) ) {
		fprintf(stderr,"can't write message(Hand handler): tossing\n");
	}
}

void ARMM_Communicator::SetObjectsData(std::vector<btRigidBody*> * obj)
{
	m_objects_body = obj;
}

void ARMM_Communicator::SetHandsData(std::vector<bt_ARMM_hand*> * hand)
{
	m_hands_body = hand;
}

#endif

