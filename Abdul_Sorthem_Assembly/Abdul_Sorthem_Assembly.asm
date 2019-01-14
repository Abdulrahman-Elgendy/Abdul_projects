%include "asm_io.inc"

SECTION .data

msg1: db "wrong number of command line arguments",0
msg2: db "your argument is not between 2 and 9",0
msg3: db "your command line argument is not between 2 and 9",0
good_msg: db "all good until now",0
arr1: db "XXXXXXXXXXXXXXXXXXXXXXX",0
msg_init: db " initial configuration",0
msg_final: db " final configuration",0
letter: db "o",0
peg: dd 0,0,0,0,0,0,0,0,0

SECTION .bss 
num_disk: resd 1

SECTION .text

	global asm_main

sorthem:
	enter 0,0
        pusha
	
        mov ebx, [ebp+8] ;address of array representing peg
        mov ecx, [ebp+12] ;number of disks

	cmp ecx, dword 1
	je Sorthem_End

	mov esi, ecx
	sub esi, dword 1
	push esi

	mov esi, dword 0

	mov esi, ebx
	add esi, dword 4
	push esi

	call sorthem   ;recursion happens here
	add esp,8
	
	mov esi, dword 0
	mov edi, dword 0
	
	mov eax, [ebx]
	cmp eax, [ebx+4]       ;checks if any modification will be needed, so it decides whether to display or not
	jge Sorthem_End  
	jmp start

	start:
		mov eax, [ebx]

		cmp esi, ecx
		je End_Loop
	
		cmp byte [ebx+4], byte 0
		je End_Loop

		cmp eax, [ebx+4]
		jge End_Loop		
		cmp eax, [ebx+4]
		jl swap

	swap:
		mov edi, ebx
		mov eax, [edi]
		mov edx, [edi+4]
		mov [edi], edx
		mov [edi+4], eax
		inc esi
		add ebx, 4
		jmp start
	End_Loop:
		;push parameters
	        mov edi, [num_disk]
                push edi

		mov edi, peg
		push edi

		call showp	
		add esp, 8

	Sorthem_End:
		popa
		leave
		ret

showp:
	enter 0,0
	pusha
	
	mov ebx, [ebp+8] ;address of array representing peg
	mov ecx, [ebp+12] ;number of disks
	mov edi, dword 1
	mov edx, ebx                              ;edx now has the number in the array, which is half the number of circles
	Go_ToFinal:
		cmp edi, ecx
		je reset
		add edx, dword 4
		inc edi
		jmp Go_ToFinal
	reset:	
		mov edi, dword 0
		jmp Line_print
	Line_print:
		cmp edi, ecx
		je final_line
		mov esi, dword 0
		mov ebx, dword 11
		sub ebx, [edx]
		print_spaces:
			cmp esi,ebx
			je reset_counter
			mov eax, ' '
			call print_char
			inc esi
			jmp print_spaces
		reset_counter:
			mov esi, dword 0
			jmp print_half1
		print_half1:		
			cmp esi, [edx]
			je print_slash
			mov eax, letter
			call print_string
			inc esi
			jmp print_half1
		print_slash:
			mov al, '|'
			call print_char
			mov esi, dword 0
			jmp print_half2
		print_half2:
			cmp esi, [edx]
			je increment
			mov eax, letter
			call print_string
			inc esi
			jmp print_half2
		increment:
			call print_nl
			inc edi
			sub edx, dword 4
			jmp Line_print	
	final_line:
		mov eax, arr1
		call print_string
		call print_nl
		call print_nl
		popa 
		leave
		call read_char
		ret
	


asm_main:
	enter 0,0	
	mov eax, dword [ebp+8]
	cmp eax, dword 2
	je Check_Cmd_Arg
	
	mov eax, msg1
	call print_string
	call print_nl
	jmp asm_main_end

	;if we pass this, then we have the right number of command line arguments
 
Check_Cmd_Arg:
		
	mov ebx, dword [ebp+12]
	mov eax, dword [ebx+4]
	cmp byte[eax], '2'
	jb Wrong_Range
	cmp byte[eax], '9'
	ja Wrong_Range
	
Check_Length:
        cmp byte[eax+1], byte 0
        je good

        mov eax, msg2
        call print_string
        call print_nl
        jmp asm_main_end

	;if we pass this, then we have the right range for the command line argument
good:
	mov ecx, peg
	mov bl , byte[eax]
	mov eax, 0
	mov al, bl
	sub eax, dword '0'
	mov [num_disk], eax
	push eax
	push ecx

	call rconf
	mov eax, msg_init
	call print_string
	call print_nl
	call print_nl
	call showp
	call sorthem
	mov eax, msg_final
	call print_string
	call print_nl
	call print_nl
	call showp
	add esp, 8

	jmp asm_main_end

Wrong_Range:
	mov eax, msg3
	call print_string
	call print_nl
	jmp asm_main_end		
asm_main_end:
	mov eax, 0		
	leave
	ret

