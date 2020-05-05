#ifndef __COROUTINE_H__
#define __COROUTINE_H__

#define CR_START() \
	static int __pc = 0; \
	switch(__pc) { \
	case 0: \

#define CR_END() \
	} \
	__pc = 0

#define CR_YIELD() \
	__pc = __LINE__; \
	return; \
	case __LINE__:

#define CR_RETURN return

#endif
