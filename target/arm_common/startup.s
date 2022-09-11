				EXPORT _STACKTOP
				EXPORT _HEAP_START
				EXPORT _HEAP_END

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
_STACKTOP

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
_HEAP_START
Heap_Mem        SPACE   Heap_Size
_HEAP_END
				END