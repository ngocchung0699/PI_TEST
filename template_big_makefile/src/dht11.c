#include <dht11.h>

uint8_t data[6];

bool dht11_read() {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;

    // pull the pin high and wait 250 milliseconds
//    digitalWrite(PIN, HIGH);
//    delay(250);

    // now pull it low for ~20 milliseconds
    pinMode(PIN, OUTPUT);
    digitalWrite(PIN, LOW);
    delay(20);
    //cli();
    digitalWrite(PIN, HIGH);
    delayMicroseconds(20);
    pinMode(PIN, INPUT);

    // read in timings
    for (i = 0; i < MAXTIMINGS; i++) {
        counter = 0;
        while (digitalRead(PIN) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) {
                break;
            }
        }
        laststate = digitalRead(PIN);

        if (counter == 255) {
            break;
        }

        // ignore first 3 transitions
        if ((i >= 4) && (i % 2 == 0)) {
            // shove each bit into the storage bytes
            data[j / 8] <<= 1;
            if (counter > count) {
                data[j / 8] |= 1;
            }
            j++;
        }
    }
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        return true;
    }
    return false;
}


float readHumidity(void) {

	float f;
	if (dht11_read()) {
		f = data[0];
		f *= 256;
		f += data[1];
		f /= 10;
		return f;
	}
    return 0;
}

float readTemperature(void) {

	float f;
	if (dht11_read()) {
		f = data[2];
		if(data[3]%128<10){
			f += data[3]%128/10.0f;
		}
        else if(data[3]%128<100){
			f += data[3]%128/100.0f;
		}
        else{
			f += data[3]%128/1000.0f;
		}
		if(data[3]>=128){ // The left-most digit indicate the negative sign. 
			f = -f;
		}
		return f;
	}
	
    return 0;
}


int readTempAndHumidity(float* data) {
    uint32_t target_val[2] = {0};
    uint32_t cnt = 0;

    data[0] = readHumidity();
    data[1] = readTemperature();
    if (isnan(data[0]) || isnan(data[1])) {
        return -1;
    }
    return 0;
}
