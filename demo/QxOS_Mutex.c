#include "cmsis_os2.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
#define TRUE true
#endif // TRUE

#ifndef FALSE
#define FALSE false
#endif //FALSE

typedef struct {
	char *name; /*!< The name of mutex */
	bool isLocked; /*!< The variable indicates if the mutex is locked. */
} tQxMutex;

typedef enum {
	QxDeviceErr,
	QxBusy,
	QxNotReady,
	QxErr,
	QxOK = 0,
} tQxStatus;

typedef struct {
	tQxMutex mutex;
	void* rtos_mutex;
} tQxMutexReference;

tQxMutex* QxOS_CreateMutex(const char* name)
{

	tQxMutexReference *pMutex = (tQxMutexReference*)malloc(sizeof(tQxMutexReference));
	memset(pMutex, 0x0, sizeof(tQxMutexReference));
	pMutex->mutex.name = (char*) name;
	pMutex->rtos_mutex = osMutexNew(NULL);
	return &pMutex->mutex;
}

tQxStatus QxOS_LockMutex(tQxMutex* mutex)
{

	tQxMutexReference *pMutex = (tQxMutexReference*) mutex;

	osMutexAcquire(pMutex->rtos_mutex, osWaitForever);

	pMutex->mutex.isLocked = TRUE;

	return QxOK;
}

tQxStatus QxOS_UnLockMutex(tQxMutex* mutex)
{

	tQxMutexReference *pMutex = (tQxMutexReference*) mutex;

	osMutexRelease(pMutex->rtos_mutex);

	pMutex->mutex.isLocked = FALSE;

	return QxOK;
}
