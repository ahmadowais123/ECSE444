	AREA text, CODE, READONLY
	export asm_dp

asm_dp
	; R0 = input vector a
	; R1 = input vector b
	; R2 = input vector size
	; R3 = output
	PUSH {R4-R7}		; Push registers to the stack to save them
	MOV R4, #0			; i = 0
loop
	LSL R5, R4, #2		; calculate offset, i*4 
	ADD R6, R5, R0		; address of a[i]
	ADD R7, R5, R1		; address of b[i]
	VLDR.F32 S3, [R6]	; a[i]
	VLDR.F32 S4, [R7]	; b[i]
	VMUL.F32 S5, S3, S4	; partialProduct = a[i]*b[i]
	VADD.F32 S6, S6, S5	; product = product + partialProduct
	ADD R4, R4, #1		; i = i + 1
	CMP R4, R2			; if i < N
	BLT loop			; loop
	VSTR S6, [R3]		; Store result in return address
	POP {R4-R7}			; Pop registers before returning
	BX LR				; Return to main
	END
	