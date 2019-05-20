/*
    Created by ReE=Rishun on 2019-05-13.
*/

#ifndef STUDYCODE_APP_H
#define STUDYCODE_APP_H

#endif //STUDYCODE_APP_H

#ifdef __cplusplus
extern "C" {
#endif


#include "target_test.h"

#define MAIN_PRIORITY   5
#define HIGH_PRIORITY   9
#define MID_PRIORITY    10
#define LOW_PRIORITY    11


#ifndef TASK_PORTID
#define TASK_PORTID     1
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define STACK_SIZE      4096
#endif /* STACK_SIZE */


#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
