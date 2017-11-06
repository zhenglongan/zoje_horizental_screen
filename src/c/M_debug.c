/*************************************************************
 * bug detect 
 ************************************************************/
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\common.h"
#include "..\..\include\M_debug.h"


#if DEBUG_DLG
void test_nop_emergency(UINT16 temp1,UINT16 temp2)
{
	UINT16 test_temp16;
	if(temp1 != 0)
	{
		test_temp16 = temp1;
		de_bug.test1 = (UINT8)(test_temp16>>8);
		de_bug.test2 = (UINT8)(test_temp16&0x00FF);
	}
	if(temp2 != 0)
	{
		test_temp16 = temp2;
		de_bug.test3 = (UINT8)(test_temp16>>8);
		de_bug.test4 = (UINT8)(test_temp16&0x00FF);
	}
}
void set_func_code_info(UINT16 temp1,UINT16 temp2)
{	
	UINT16 test_temp16;
	
	test_temp16 = temp1;
	de_bug.test1 = (UINT8)(test_temp16>>8);
	de_bug.test2 = (UINT8)(test_temp16&0x00FF);
	
	test_temp16 = temp2;
	de_bug.test3 = (UINT8)(test_temp16>>8);
	de_bug.test4 = (UINT8)(test_temp16&0x00FF);

}
void set_protocol_usrful(UINT8 first,UINT8 second)
{
	de_bug.test3 = first;
	de_bug.test4 = second;
}
#endif 