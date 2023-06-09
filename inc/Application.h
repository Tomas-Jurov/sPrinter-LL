/*
 * Application.h
 *
 *  Created on: Jun 17, 2018
 *      Author: xgallom
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "stm32f4xx.h"
#include "Periph/Led.h"
#include "Periph/Engine.h"
#include "Periph/Servo.h"
#include "Util/Control.h"
#include "Periph/Encoder.h"
#include "Util/Pid.h"


class Application {
	static Application *m_instance;

	class ApplicationInitializator {
	public:
		ApplicationInitializator(Application *parent);
	} m_applicationInitializator;


public:
	Util::Control ctrl;

private:
	Periph::Led m_appRunningLed;


public:
	Application();

	void run();
	static Application *instance();

};

#define App ::Application::instance()

#endif /* APPLICATION_H_ */
