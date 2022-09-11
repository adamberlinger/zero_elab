Stack_Size		EQU     0x200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
_STACKTOP
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size      EQU     2560

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
_HEAP_START
Heap_Mem        SPACE   Heap_Size
_HEAP_END

	EXPORT  __initial_sp
	EXPORT  _HEAP_START
	EXPORT  _HEAP_END
	EXPORT  _STACKTOP

	END
