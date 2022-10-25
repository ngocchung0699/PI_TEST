#include <dht11.h>

int dht11_dat[5] = { 0, 0, 0, 0, 0 };
 
bool read_dht11(float *data)
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j		= 0, i;
	float	f; 

	dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;
 
	pinMode( DHTPIN, OUTPUT );
	digitalWrite( DHTPIN, LOW );
	delay( 18 );
	digitalWrite( DHTPIN, HIGH );
	delayMicroseconds( 40 );
	pinMode( DHTPIN, INPUT );

	for ( i = 0; i < MAXTIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead( DHTPIN ) == laststate )
		{
			counter++;
			delayMicroseconds( 1 );
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( DHTPIN );
 
		if ( counter == 255 )
			break;
 
		if ( (i >= 4) && (i % 2 == 0) )
		{
			dht11_dat[j / 8] <<= 1;
			if ( counter > 16 )
				dht11_dat[j / 8] |= 1;
			j++;
		}
	}

	if ( (j >= 40) && (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) ){

		// if(dht11_dat[1]/10 > 0){	data[0] = dht11_dat[0] + dht11_dat[1]*0.01;}
		// else{ 						data[0] = dht11_dat[0] + dht11_dat[1]*0.1;}

		// if(dht11_dat[3]/10 > 0){	data[1] = dht11_dat[2] + dht11_dat[3]*0.01;}	// (F = C * 9. / 5. + 32)
		// else{ 						data[1] = dht11_dat[2] + dht11_dat[3]*0.1;}  
		data[0] = dht11_dat[0];
		data[1] = dht11_dat[2];
		return true;
	}
	else  {
		return false;
	}
}



