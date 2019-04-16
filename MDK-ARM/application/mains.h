#ifndef _MAINS_H_
#define _MAINS_H_

#ifdef __cplusplus
 extern "C" {
	 #include "gimbal.h"
	 #include "shoot_task.h"
	 #include "control_task.h"
	 #include "keymouse_task.h"
#endif

#define imca_msg 0
#define skyguard_msg 1

void SETUP(void);
void loop(void);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
