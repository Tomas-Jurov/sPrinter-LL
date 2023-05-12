/*
 * EncoderGroup.h
 *
 *  Created on: Apr 8, 2023
 *      Author: bugino
 */

#ifndef PERIPH_ENCODERGROUP_H_
#define PERIPH_ENCODERGROUP_H_

#include "Periph/Encoder.h"
// #include "Util/State.h"

namespace Periph {

class EncoderGroup {

	Encoder *m_encoders[3];
	uint8_t m_numOfEncoders;

public:
	EncoderGroup(Encoder *encoder1, Encoder *encoder2, Encoder *encoder3, uint8_t numOfEncoders);

	void update();

	void reset();

	int32_t getDistance();
	uint8_t getAngularSpeed();
	uint8_t getAngularSpeedInScale();
};

}

#endif /* PERIPH_ENCODERGROUP_H_ */
