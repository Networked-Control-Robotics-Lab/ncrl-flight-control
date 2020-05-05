#ifndef __COROUTINE_H__
#define __COROUTINE_H__

#define cr_start() \
	static int __pc = 0; \
	switch(__pc) { \
	case 0: \

#define cr_end() \
	} \
	__pc = 0

#define cr_yield() \
	__pc = __LINE__; \
	return; \
	case __LINE__:

#endif
