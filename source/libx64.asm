; x64 ASM library for memory benchmarks
; Note order: RDI, RSI, RDX, RCX, R8, R9

format ELF64
public GetCPUinfo  as 'GetCPUinfo'
public ReadSSE128  as 'ReadSSE128'
public WriteSSE128 as 'WriteSSE128'
public CopySSE128  as 'CopySSE128'
public ReadAVX256  as 'ReadAVX256'
public WriteAVX256 as 'WriteAVX256'
public CopyAVX256  as 'CopyAVX256'

ITERATIONS    EQU 1000000  ; Number of measurement iterations
SYS_NANOSLEEP EQU 35       ; API function number

; Get CPU features, measure TSC clock
; INPUT:
; parm#1 = RDI = pointer for return methods bitmap
; parm#2 = RSI = pointer for return TSC clock, 0 means not available
; parm#3 = RDX = reserved for CPU context bitmap pointer, not used yet
; parm#4 = RCX = reserved for OS context bitmap pointer, not used yet
; OUTPUT:
; RAX = 0, yet reserved for status return
;
GetCPUinfo:

push rbx              ; RBX must be non-volaile by convention
xor eax,eax
mov [rdi],rax         ; pre-clear bitmap
mov [rsi],rax         ; pre-clear clock frequency
;--- Detect CPUID ---
call CheckCPUID
jc .L0                ; Go skip if CPUID not supported
cmp eax,1
jb .L0                ; Go skip if CPUID function 1 not supported
mov eax,1
cpuid
;--- Check SSE support ---
bt edx,25
jnc @f
or byte [rdi],00000001b  ; Bitmap[0] = SSE
@@:
;--- Check AVX support ---
mov eax,018000000h       ; This mask for bits 27, 28
and ecx,eax              ; ECX = Part of features bitmaps
cmp ecx,eax
jne @f                   ; Go if OSXSAVE(ECX.27) or AVX(ECX.28) not supported
;--- Check AVX context management option in the XCR0 ---
xor ecx,ecx              ; ECX = XCR register index
xgetbv                   ; Read XCR0 to EDX:EAX
and al,00000110b         ; This mask for bits 1, 2
cmp al,00000110b
jne @f                   ; Go if AVX context not initialized by OS
or byte [rdi],00000010b  ; Bitmap[1] = AVX256
@@:
;--- Check and measure TSC clock ---
mov eax,1
cpuid
test dl,00010000b        ; Check TSC support
jz .L0                   ; Go skip if TSC not supported
call MeasureCpuClk
jc .L0                   ; Go skip if measurement error
mov [rsi],rax
;--- Done ---
.L0:
pop rbx
xor eax,eax
ret

; Read memory benchmark, SSE128
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = reserved for copy function, not used
; OUTPUT:
; RAX = 0, yet reserved for status return
;
ReadSSE128:

mov r10,rdx
rdtsc                   ; load time stamp counter to EAX=low, EDX=high
mov r8d,eax
mov r9d,edx
mov r11,ITERATIONS
shr ecx,8               ; 16 instructions , 16 bytes per instruction, /256

.L0:                    ; This cycle for measurements
mov eax,ecx
mov rdx,r10

@@:                     ; This cycle is 256 bytes per one iteration
movaps xmm0, [rdx+000]
movaps xmm1, [rdx+016]
movaps xmm2, [rdx+032]
movaps xmm3, [rdx+048]
movaps xmm4, [rdx+064]
movaps xmm5, [rdx+080]
movaps xmm6, [rdx+096]
movaps xmm7, [rdx+112]
movaps xmm8, [rdx+128]
movaps xmm9, [rdx+144]
movaps xmm10,[rdx+160]
movaps xmm11,[rdx+176]
movaps xmm12,[rdx+192]
movaps xmm13,[rdx+208]
movaps xmm14,[rdx+224]
movaps xmm15,[rdx+240]

add rdx,16*16
dec eax
jnz @b      ; cycle for 256 bytes per iteration

dec r11
jnz .L0     ; cycle for measurement

rdtsc       ; load time stamp counter to EAX=low, EDX=high
sub eax,r8d
sbb edx,r9d
mov dword [rdi+0],eax        ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx        ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16
mov qword [rsi+0],rax        ; number of MOVAPS instructions executed
xor eax,eax
ret

; Write memory benchmark, SSE128
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = reserved for copy function, not used
; OUTPUT:
; RAX = 0, yet reserved for status return
;
WriteSSE128:

xorps xmm0,xmm0    ; required clear because write to memory
xorps xmm1,xmm1
xorps xmm2,xmm2
xorps xmm3,xmm3
xorps xmm4,xmm4
xorps xmm5,xmm5
xorps xmm6,xmm6
xorps xmm7,xmm7
xorps xmm8,xmm8
xorps xmm9,xmm9
xorps xmm10,xmm10
xorps xmm11,xmm11
xorps xmm12,xmm12
xorps xmm13,xmm13
xorps xmm14,xmm14
xorps xmm15,xmm15

mov r10,rdx
rdtsc                  ; load time stamp counter to EAX=low, EDX=high
mov r8d,eax
mov r9d,edx
mov r11,ITERATIONS
shr ecx,8              ; 16 instructions , 16 bytes per instruction

.L0:                   ; cycle for measurements
mov eax,ecx
mov rdx,r10

@@:                    ; cycle for 256 bytes per iteration
movaps [rdx+000],xmm0
movaps [rdx+016],xmm1
movaps [rdx+032],xmm2
movaps [rdx+048],xmm3
movaps [rdx+064],xmm4
movaps [rdx+080],xmm5
movaps [rdx+096],xmm6
movaps [rdx+112],xmm7
movaps [rdx+128],xmm8
movaps [rdx+144],xmm9
movaps [rdx+160],xmm10
movaps [rdx+176],xmm11
movaps [rdx+192],xmm12
movaps [rdx+208],xmm13
movaps [rdx+224],xmm14
movaps [rdx+240],xmm15

add rdx,16*16
dec eax
jnz @b

dec r11
jnz .L0

rdtsc                        ; load time stamp counter to EAX=low, EDX=high
sub eax,r8d
sbb edx,r9d
mov dword [rdi+0],eax        ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx        ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16
mov qword [rsi+0],rax        ; number of MOVAPS instructions executed
xor eax,eax
ret

; Copy memory benchmark, SSE128
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target SOURCE array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = pointer to target DESTINATION array
; OUTPUT:
; RAX = 0, yet reserved for status return
;
CopySSE128:

push rbx r12
mov r10,rdx
rdtsc
mov r12d,eax
mov r9d,edx
mov r11,ITERATIONS
shr ecx,8              ; 16 instructions , 16 bytes per instruction

.L0:                   ; cycle for measurements
mov eax,ecx
mov rdx,r10            ; RDX = load source pointer
mov rbx,r8             ; RBX = load destination pointer

@@:                    ; cycle for 256 bytes per iteration
movaps xmm0, [rdx+000]
movaps xmm1, [rdx+016]
movaps xmm2, [rdx+032]
movaps xmm3, [rdx+048]
movaps xmm4, [rdx+064]
movaps xmm5, [rdx+080]
movaps xmm6, [rdx+096]
movaps xmm7, [rdx+112]
movaps xmm8, [rdx+128]
movaps xmm9, [rdx+144]
movaps xmm10,[rdx+160]
movaps xmm11,[rdx+176]
movaps xmm12,[rdx+192]
movaps xmm13,[rdx+208]
movaps xmm14,[rdx+224]
movaps xmm15,[rdx+240]

movaps [rbx+000],xmm0
movaps [rbx+016],xmm1
movaps [rbx+032],xmm2
movaps [rbx+048],xmm3
movaps [rbx+064],xmm4
movaps [rbx+080],xmm5
movaps [rbx+096],xmm6
movaps [rbx+112],xmm7
movaps [rbx+128],xmm8
movaps [rbx+144],xmm9
movaps [rbx+160],xmm10
movaps [rbx+176],xmm11
movaps [rbx+192],xmm12
movaps [rbx+208],xmm13
movaps [rbx+224],xmm14
movaps [rbx+240],xmm15

add rdx,16*16
add rbx,16*16
dec eax
jnz @b              ; cycle for 256 bytes per iteration

dec r11
jnz .L0             ; cycle for measurements

rdtsc               ; load time stamp counter to EAX=low, EDX=high
sub eax,r12d
sbb edx,r9d
mov dword [rdi+0],eax         ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx         ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16
mov qword [rsi+0],rax     ; number of copy-pairs MOVAPS instructions executed
xor eax,eax
pop r12 rbx
ret

; Read memory benchmark, AVX256
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = reserved for copy function, not used
; OUTPUT:
; RAX = 0, yet reserved for status return
;
ReadAVX256:

mov r10,rdx
rdtsc                   ; load time stamp counter to EAX=low, EDX=high
mov r8d,eax
mov r9d,edx
mov r11,ITERATIONS      ; Measurement iterations
shr ecx,9               ; 16 instructions , 32 bytes per instruction, /512

.L0:                    ; This cycle for measurements
mov eax,ecx
mov rdx,r10

@@:                     ; This cycle is 512 bytes per one iteration
vmovapd ymm0, [rdx+32*00]
vmovapd ymm1, [rdx+32*01]
vmovapd ymm2, [rdx+32*02]
vmovapd ymm3, [rdx+32*03]
vmovapd ymm4, [rdx+32*04]
vmovapd ymm5, [rdx+32*05]
vmovapd ymm6, [rdx+32*06]
vmovapd ymm7, [rdx+32*07]
vmovapd ymm8, [rdx+32*08]
vmovapd ymm9, [rdx+32*09]
vmovapd ymm10,[rdx+32*10]
vmovapd ymm11,[rdx+32*11]
vmovapd ymm12,[rdx+32*12]
vmovapd ymm13,[rdx+32*13]
vmovapd ymm14,[rdx+32*14]
vmovapd ymm15,[rdx+32*15]

add rdx,16*32
dec eax
jnz @b      ; cycle for 512 bytes per iteration

dec r11
jnz .L0     ; cycle for measurement

rdtsc       ; load time stamp counter to EAX=low, EDX=high
sub eax,r8d
sbb edx,r9d
mov dword [rdi+0],eax        ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx        ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16   ; 16 instructions per one pass
mov qword [rsi+0],rax        ; number of VMOVAPD instructions executed
xor eax,eax
ret

; Write memory benchmark, AVX256
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = reserved for copy function, not used
; OUTPUT:
; RAX = 0, yet reserved for status return
;
WriteAVX256:

vxorpd ymm0,ymm0,ymm0    ; required clear because write to memory
vxorpd ymm1,ymm1,ymm1
vxorpd ymm2,ymm2,ymm2
vxorpd ymm3,ymm3,ymm3
vxorpd ymm4,ymm4,ymm4
vxorpd ymm5,ymm5,ymm5
vxorpd ymm6,ymm6,ymm6
vxorpd ymm7,ymm7,ymm7
vxorpd ymm8,ymm8,ymm8
vxorpd ymm9,ymm9,ymm9
vxorpd ymm10,ymm10,ymm10
vxorpd ymm11,ymm11,ymm11
vxorpd ymm12,ymm12,ymm12
vxorpd ymm13,ymm13,ymm13
vxorpd ymm14,ymm14,ymm14
vxorpd ymm15,ymm15,ymm15

mov r10,rdx
rdtsc                  ; load time stamp counter to EAX=low, EDX=high
mov r8d,eax
mov r9d,edx
mov r11,ITERATIONS
shr ecx,9              ; 16 instructions , 32 bytes per instruction

.L0:                   ; cycle for measurements
mov eax,ecx
mov rdx,r10

@@:                    ; cycle for 512 bytes per iteration
vmovapd [rdx+32*00],ymm0
vmovapd [rdx+32*01],ymm1
vmovapd [rdx+32*02],ymm2
vmovapd [rdx+32*03],ymm3
vmovapd [rdx+32*04],ymm4
vmovapd [rdx+32*05],ymm5
vmovapd [rdx+32*06],ymm6
vmovapd [rdx+32*07],ymm7
vmovapd [rdx+32*08],ymm8
vmovapd [rdx+32*09],ymm9
vmovapd [rdx+32*10],ymm10
vmovapd [rdx+32*11],ymm11
vmovapd [rdx+32*12],ymm12
vmovapd [rdx+32*13],ymm13
vmovapd [rdx+32*14],ymm14
vmovapd [rdx+32*15],ymm15

add rdx,16*32
dec eax
jnz @b

dec r11
jnz .L0

rdtsc                        ; load time stamp counter to EAX=low, EDX=high
sub eax,r8d
sbb edx,r9d
mov dword [rdi+0],eax        ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx        ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16
mov qword [rsi+0],rax        ; number of VMOVAPD instructions executed
xor eax,eax
ret

; Copy memory benchmark, AVX256
; INPUT:
; parm#1 = RDI = pointer for return delta TSC
; parm#2 = RSI = pointer for return number of target instructions
; parm#3 = RDX = pointer to target SOURCE array
; parm#4 = RCX = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = R8  = pointer to target DESTINATION array
; OUTPUT:
; RAX = 0, yet reserved for status return
;
CopyAVX256:

push rbx r12
mov r10,rdx
rdtsc
mov r12d,eax
mov r9d,edx
mov r11,ITERATIONS
shr ecx,9              ; 16 instructions , 32 bytes per instruction

.L0:                   ; cycle for measurements
mov eax,ecx
mov rdx,r10            ; RDX = load source pointer
mov rbx,r8             ; RBX = load destination pointer

@@:                    ; cycle for 512 bytes per iteration
vmovapd ymm0, [rdx+32*00]
vmovapd ymm1, [rdx+32*01]
vmovapd ymm2, [rdx+32*02]
vmovapd ymm3, [rdx+32*03]
vmovapd ymm4, [rdx+32*04]
vmovapd ymm5, [rdx+32*05]
vmovapd ymm6, [rdx+32*06]
vmovapd ymm7, [rdx+32*07]
vmovapd ymm8, [rdx+32*08]
vmovapd ymm9, [rdx+32*09]
vmovapd ymm10,[rdx+32*10]
vmovapd ymm11,[rdx+32*11]
vmovapd ymm12,[rdx+32*12]
vmovapd ymm13,[rdx+32*13]
vmovapd ymm14,[rdx+32*14]
vmovapd ymm15,[rdx+32*15]

vmovapd [rbx+32*00],ymm0
vmovapd [rbx+32*01],ymm1
vmovapd [rbx+32*02],ymm2
vmovapd [rbx+32*03],ymm3
vmovapd [rbx+32*04],ymm4
vmovapd [rbx+32*05],ymm5
vmovapd [rbx+32*06],ymm6
vmovapd [rbx+32*07],ymm7
vmovapd [rbx+32*08],ymm8
vmovapd [rbx+32*09],ymm9
vmovapd [rbx+32*10],ymm10
vmovapd [rbx+32*11],ymm11
vmovapd [rbx+32*12],ymm12
vmovapd [rbx+32*13],ymm13
vmovapd [rbx+32*14],ymm14
vmovapd [rbx+32*15],ymm15

add rdx,16*32
add rbx,16*32
dec eax
jnz @b              ; cycle for 512 bytes per iteration

dec r11
jnz .L0             ; cycle for measurements

rdtsc               ; load time stamp counter to EAX=low, EDX=high
sub eax,r12d
sbb edx,r9d
mov dword [rdi+0],eax         ; delta-TSC per second (frequency), low dword
mov dword [rdi+4],edx         ; delta-TSC per second (frequency), high dword

imul rax,rcx,ITERATIONS*16
mov qword [rsi+0],rax     ; number of copy-pairs MOVAPS instructions executed
xor eax,eax
pop r12 rbx
ret


;---------- Helpers methods ----------------------------------------------------

;------------------------------------------------------------------------;
; Check CPUID instruction support.                                       ;
; Changes RAX, RBX, RCX, RDX                                             ;
;                                                                        ;
; INPUT:   None                                                          ;
;                                                                        ;
; OUTPUT:  CF = Error flag,                                              ; 
;          0(NC) = Result in EAX valid, 1(C) = Result not valid          ;
;          EAX = Maximum supported standard function, if no errors       ;
;------------------------------------------------------------------------;
CheckCPUID:
mov ebx,21
pushf                     ; In the 64-bit mode, push RFLAGS
pop rax
bts eax,ebx               ; Set EAX.21=1
push rax
popf                      ; Load RFLAGS with RFLAGS.21=1
pushf                     ; Store RFLAGS
pop rax                   ; Load RFLAGS to RAX
btr eax,ebx               ; Check EAX.21=1, Set EAX.21=0
jnc .L0                   ; Go error branch if cannot set EFLAGS.21=1
push rax
popf                      ; Load RFLAGS with RFLAGS.21=0
pushf                     ; Store RFLAGS
pop rax                   ; Load RFLAGS to RAX
btr eax,ebx               ; Check EAX.21=0
jc .L0                    ; Go if cannot set EFLAGS.21=0
xor eax,eax
cpuid
ret
.L0:
stc
ret

;--- Measure CPU TSC (Time Stamp Counter) clock frequency ---------------;
;                                                                        ;
; INPUT:   None                                                          ;
;                                                                        ;
; OUTPUT:  CF flag = Status: 0(NC)=Measured OK, 1(C)=Measurement error	 ;
;          Output RAX valid only if CF=0(NC)                             ;
;          RAX = TSC Frequency, Hz, F = Delta TSC per 1 second           ;
;------------------------------------------------------------------------;
MeasureCpuClk:
push rcx rsi rdi r8 r9 r10 r11
;--- Prepare parameters, early to minimize dTSC ---
;lea rdi,[TimespecWait]    ; RDI = Pointer to loaded wait time: DQ sec, ns
;lea rsi,[rdi+16]          ; RSI = Pointer to stored remain time: DQ sec, ns
sub rsp,32
mov rdi,rsp
lea rsi,[rdi+16]
xor eax,eax
mov qword [rdi+00],1
mov qword [rdi+08],rax
mov qword [rsi+00],rax
mov qword [rsi+08],rax
;--- Get TSC value before 1 second pause ---
rdtsc                     ; EDX:EAX = TSC, EDX = High , EAX = Low
push rax rdx
;--- Wait 1 second ---
mov eax,SYS_NANOSLEEP     ; EAX = Linux API function (syscall number)
push rsi
syscall
pop rsi
xchg r8,rax
;--- Get TSC value after 1 second pause ---
rdtsc                     ; EDX:EAX = TSC, EDX = High , EAX = Low , BEFORE 1 second pause
pop rcx rdi               ; ECX:EDI = TSC, ECX = High , EBX = Low , AFTER 1 second pause
;--- Check results ---
test r8,r8
jnz TimerFailed           ; Go if error returned or wait interrupted
mov r8,[rsi+00]           ; RAX = Time remain, seconds
or  r8,[rsi+08]           ; RAX = Disjunction with Time remain, nanoseconds
jnz TimerFailed           ; Go if remain time stored by function
;--- Calculate delta-TSC per 1 second = TSC frequency ---
sub eax,edi               ; Subtract: DeltaTSC.Low  = EndTSC.Low - StartTSC.Low
sbb edx,ecx               ; Subtract: DeltaTSC.High = EndTSC.High - StartTSC.High - Borrow
;--- Extract TSC frequency as 64-bit value ---
shl rdx,32
add rax,rdx
;--- Exit points ---
add rsp,32
clc
TimerDone:
pop r11 r10 r9 r8 rdi rsi rcx
ret
TimerFailed:
add rsp,32
stc
jmp TimerDone

