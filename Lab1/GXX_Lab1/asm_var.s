	AREA text, CODE, READONLY
	export asm_var

asm_var
	; R0 input array
	; R1 input array size
	; R2 output value
	
	PUSH {R4-R5}
	MOV R4, #0				; i = 0
	VSUB.F32 S2, S2, S2		; Set S2 to 0, will hold mean
mean
	LSL R5, R4, #2			; calculate offset
	ADD R5, R0, R5			; calculate address of next element
	VLDR.F32 S1, [R5]		; load element in fp register
	VADD.F32 S2, S2, S1		; add element to total sum
	ADD R4, R4, #1			; i = i + 1
	CMP R4, R1				; if i < 1000
	BLT mean				; loop
	VMOV.F32 S3, R1			; Move integer value to fp register
	VCVT.F32.U32 S3, S3		; Convert int to floating point
	VDIV.F32 S2, S2, S3		; S2 := mean, mean = sum/size
	
	MOV R4, #0				; i = 0
	VSUB.F32 S4, S4, S4		; Set S4 to 0, will hold variance
variance
	LSL R5, R4, #2			; calculate offset
	ADD R5, R0, R5			; calculate address of next element
	VLDR.F32 S1, [R5]		; load element in fp register
	VSUB.F32 S5, S1, S2		; subtract mean from element
	VMUL.F32 S5, S5, S5		; calculate the square of the difference
	VADD.F32 S4, S4, S5		; add the result to the intermediate total
	ADD R4, R4, #1			; i = i + 1
	CMP R4, R1				; if i < 1000
	BLT variance			; loop 
	SUB R1, R1, #1			; N = N-1
	VMOV.F32 S3, R1			; Move int value to fp register
	VCVT.F32.U32 S3, S3		; Convert int to floating point
	VDIV.F32 S4, S4, S3		; S4 : = variance, variance = [total of all (a[i]-mean)]/N-1
	VSTR.F32 S4, [R2]		; Store result in return address
	
	POP {R4-R5}
	BX LR
	END