; ia32 ASM library for memory benchmarks

format ELF
public GetCPUinfo  as 'GetCPUinfo'
public ReadSSE128  as 'ReadSSE128'
public WriteSSE128 as 'WriteSSE128'
public CopySSE128  as 'CopySSE128'
public ReadAVX256  as 'ReadAVX256'
public WriteAVX256 as 'WriteAVX256'
public CopyAVX256  as 'CopyAVX256'

ITERATIONS EQU 1000000     ; Number of measurement iterations
SYS_NANOSLEEP EQU 35       ; API function number

; Get CPU features, measure TSC clock
; INPUT:
; parm#1 = pointer for return methods bitmap
; parm#2 = pointer for return TSC clock, 0 means not available
; parm#3 = reserved for CPU context bitmap pointer, not used yet
; parm#4 = reserved for OS context bitmap pointer, not used yet
; OUTPUT:
; EAX = 0, yet reserved for status return
;
GetCPUinfo:
push ebx ebp esi edi
lea ebp,[esp+20]  ; This means first parameter at [ebp+00]
mov edi,[ebp+00]
mov esi,[ebp+04]

xor eax,eax
mov [edi+00],eax
mov [edi+04],eax
mov [esi+00],eax
mov [esi+04],eax

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
or byte [edi],00000001b  ; Bitmap[0] = SSE
@@:

;--- Check and measure TSC clock ---
test dl,00010000b
jz .L0                ; Go skip if TSC not supported
call MeasureCpuClk
jc .L0                ; Go skip if measurement error
mov [esi+00],eax
mov [esi+04],edx

;--- Done ---
.L0:
xor eax,eax
pop edi esi ebp ebx
ret


; Read memory benchmark, SSE128
; INPUT:
; parm#1 = pointer for return delta TSC
; parm#2 = pointer for return number of target instructions
; parm#3 = pointer to target array
; parm#4 = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = reserved for copy function, not used
; OUTPUT:
; EAX = 0, yet reserved for status return
;
ReadSSE128:
push ebx ebp esi edi
lea ebp,[esp+20]  ; This means first parameter at [ebp+00]

rdtsc             ; load time stamp counter to EAX=low, EDX=high
mov ebx,eax
mov ecx,edx

mov eax,[ebp+12]
shr eax,8
mov esi,ITERATIONS

.L0:                   ; this cycle for measurement
mov edx,eax
mov edi,[ebp+08]

@@:                    ; this cycle for 256 bytes per iteration
movaps xmm0,[edi+000]
movaps xmm1,[edi+016]
movaps xmm2,[edi+032]
movaps xmm3,[edi+048]
movaps xmm4,[edi+064]
movaps xmm5,[edi+080]
movaps xmm6,[edi+096]
movaps xmm7,[edi+112]
movaps xmm0,[edi+128]
movaps xmm1,[edi+144]
movaps xmm2,[edi+160]
movaps xmm3,[edi+176]
movaps xmm4,[edi+192]
movaps xmm5,[edi+208]
movaps xmm6,[edi+224]
movaps xmm7,[edi+240]

add edi,16*16
dec edx
jnz @b

dec esi
jnz .L0

rdtsc              ; load time stamp counter to EAX=low, EDX=high
sub eax,ebx
sbb edx,ecx
mov ebx,[ebp+00]
mov [ebx+00],eax   ; delta-TSC per second (frequency), low dword
mov [ebx+04],edx   ; delta-TSC per second (frequency), high dword

mov eax,[ebp+12]
mov ebx,ITERATIONS
mul ebx
shrd eax,edx,4    ; /16 means convert bytes to 128-bit instructions
shr edx,4
mov ebx,[ebp+04]
mov [ebx+00],eax
mov [ebx+04],edx

xor eax,eax
pop edi esi ebp ebx
ret

; Write memory benchmark, SSE128
; INPUT:
; parm#1 = pointer for return delta TSC
; parm#2 = pointer for return number of target instructions
; parm#3 = pointer to target array
; parm#4 = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = reserved for copy function, not used
; OUTPUT:
; EAX = 0, yet reserved for status return
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

push ebx ebp esi edi
lea ebp,[esp+20]  ; This means first parameter at [ebp+00]

rdtsc             ; load time stamp counter to EAX=low, EDX=high
mov ebx,eax
mov ecx,edx

mov eax,[ebp+12]
shr eax,8
mov esi,ITERATIONS

.L0:                   ; this cycle for measurement
mov edx,eax
mov edi,[ebp+08]

@@:                    ; this cycle for 256 bytes per iteration
movaps [edi+000],xmm0
movaps [edi+016],xmm1
movaps [edi+032],xmm2
movaps [edi+048],xmm3
movaps [edi+064],xmm4
movaps [edi+080],xmm5
movaps [edi+096],xmm6
movaps [edi+112],xmm7
movaps [edi+128],xmm0
movaps [edi+144],xmm1
movaps [edi+160],xmm2
movaps [edi+176],xmm3
movaps [edi+192],xmm4
movaps [edi+208],xmm5
movaps [edi+224],xmm6
movaps [edi+240],xmm7

add edi,16*16
dec edx
jnz @b

dec esi
jnz .L0

rdtsc              ; load time stamp counter to EAX=low, EDX=high
sub eax,ebx
sbb edx,ecx
mov ebx,[ebp+00]
mov [ebx+00],eax   ; delta-TSC per second (frequency), low dword
mov [ebx+04],edx   ; delta-TSC per second (frequency), high dword

mov eax,[ebp+12]
mov ebx,ITERATIONS
mul ebx
shrd eax,edx,4     ; /16 means convert bytes to 128-bit instructions
shr edx,4
mov ebx,[ebp+04]
mov [ebx+00],eax   ; number of MOVAPS instructions executed, low dword
mov [ebx+04],edx   ; number of MOVAPS instructions executed, high dword

xor eax,eax
pop edi esi ebp ebx
ret


; Copy memory benchmark, SSE128
; INPUT:
; parm#1 = pointer for return delta TSC
; parm#2 = pointer for return number of target instructions
; parm#3 = pointer to target SOURCE array
; parm#4 = size of target array, bytes, low 32 bit only (ECX)
; parm#5 = pointer to target DESTINATION array
; OUTPUT:
; EAX = 0, yet reserved for status return
;
CopySSE128:
push ebx ebp esi edi
lea ebp,[esp+20]  ; This means first parameter at [ebp+00]

rdtsc             ; load time stamp counter to EAX=low, EDX=high
mov ebx,eax
mov ecx,edx

mov eax,ITERATIONS

.L0:              ; this cycle for measurement
mov edx,[ebp+12]  ; parm#4 = size
shr edx,8         ; size/256
mov esi,[ebp+08]  ; parm#3 = source pointer
mov edi,[ebp+16]  ; parm#5 = destination pointer

@@:                    ; this cycle 256 bytes per iteration
movaps xmm0,[esi+000]
movaps xmm1,[esi+016]
movaps xmm2,[esi+032]
movaps xmm3,[esi+048]
movaps xmm4,[esi+064]
movaps xmm5,[esi+080]
movaps xmm6,[esi+096]
movaps xmm7,[esi+112]

movaps [edi+000],xmm0
movaps [edi+016],xmm1
movaps [edi+032],xmm2
movaps [edi+048],xmm3
movaps [edi+064],xmm4
movaps [edi+080],xmm5
movaps [edi+096],xmm6
movaps [edi+112],xmm7

movaps xmm0,[esi+128]
movaps xmm1,[esi+144]
movaps xmm2,[esi+160]
movaps xmm3,[esi+176]
movaps xmm4,[esi+192]
movaps xmm5,[esi+208]
movaps xmm6,[esi+224]
movaps xmm7,[esi+240]

movaps [edi+128],xmm0
movaps [edi+144],xmm1
movaps [edi+160],xmm2
movaps [edi+176],xmm3
movaps [edi+192],xmm4
movaps [edi+208],xmm5
movaps [edi+224],xmm6
movaps [edi+240],xmm7

add esi,16*16
add edi,16*16
dec edx
jnz @b

dec eax
jnz .L0

rdtsc             ; load time stamp counter to EAX=low, EDX=high
sub eax,ebx
sbb edx,ecx
mov ebx,[ebp+00]
mov [ebx+00],eax  ; delta-TSC per second (frequency), low dword
mov [ebx+04],edx  ; delta-TSC per second (frequency), high dword

mov eax,[ebp+12]
mov ebx,ITERATIONS
mul ebx
shrd eax,edx,4    ; /16 means convert bytes to 128-bit instructions
shr edx,4
mov ebx,[ebp+04]
mov [ebx+00],eax  ; num. of copy-pairs MOVAPS instructions executed, low dword
mov [ebx+04],edx  ; num. of copy-pairs MOVAPS instructions executed, high dword

xor eax,eax
pop edi esi ebp ebx
ret

; Reserved functions, AVX256 at ia32 mode not supported yet, only for x64.

ReadAVX256:
WriteAVX256:
CopyAVX256:
mov eax,1
ret

;---------- Helpers methods ----------------------------------------------------

;------------------------------------------------------------------------;
; Check CPUID instruction support.                                       ;
; Changes EAX, EBX, ECX, EDX                                             ;
;                                                                        ;
; INPUT:   None                                                          ;
;                                                                        ;
; OUTPUT:  CF = Error flag,                                              ; 
;          0(NC) = Result in EAX valid, 1(C) = Result not valid          ;
;          EAX = Maximum supported standard function, if no errors       ;
;------------------------------------------------------------------------;
CheckCPUID:
mov ebx,21
pushf                     ; In the 32-bit mode, push EFLAGS
pop eax
bts eax,ebx               ; Set EAX.21=1
push eax
popf                      ; Load EFLAGS with EFLAGS.21=1
pushf                     ; Store EFLAGS
pop eax                   ; Load EFLAGS to EAX
btr eax,ebx               ; Check EAX.21=1, Set EAX.21=0
jnc .L0                   ; Go error branch if cannot set EFLAGS.21=1
push eax
popf                      ; Load EFLAGS with EFLAGS.21=0
pushf                     ; Store EFLAGS
pop eax                   ; Load EFLAGS to EAX
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
;          Output RAX,RDX valid only if CF=0(NC)                         ;
;          EDX:EAX = TSC Frequency, Hz, F = Delta TSC per 1 second       ;
;------------------------------------------------------------------------;
MeasureCpuClk:
push edi esi ebp
;--- Prepare parameters, early to minimize dTSC ---
; lea ebx,[TimespecWait]  ; EBX = Pointer to loaded wait time: DQ sec, ns
sub esp,32
mov ebx,esp
lea ecx,[ebx+16]          ; ECX = Pointer to stored remain time: DQ sec, ns
xor eax,eax
mov dword [ebx+00],1
mov dword [ebx+04],eax 
mov dword [ebx+08],eax
mov dword [ebx+12],eax 
mov dword [ecx+00],eax
mov dword [ecx+04],eax 
mov dword [ecx+08],eax
mov dword [ecx+12],eax 
;--- Get TSC value before 1 second pause ---
rdtsc                     ; EDX:EAX = TSC, EDX = High , EAX = Low
push eax edx
;--- Wait 1 second ---
mov eax,162               ; EAX = Linux API function (syscall number) = SYS_NANOSLEEP
push ecx
int 80h
pop ecx
xchg ebx,eax
;--- Get TSC value after 1 second pause ---
rdtsc                     ; EDX:EAX = TSC, EDX = High , EAX = Low , BEFORE 1 second pause
pop edi esi               ; EDI:ESI = TSC, ECX = High , EBX = Low , AFTER 1 second pause
;--- Check results ---
test ebx,ebx
jnz TimerFailed           ; Go if error returned or wait interrupted
mov ebx,[ecx+00]          ; Time remain, seconds
or ebx,[ecx+04]
or ebx,[ecx+08]           ; Disjunction with Time remain, nanoseconds
or ebx,[ecx+12]
jnz TimerFailed           ; Go if remain time stored by function
;--- Calculate delta-TSC per 1 second = TSC frequency ---
sub eax,esi               ; Subtract: DeltaTSC.Low  = EndTSC.Low - StartTSC.Low
sbb edx,edi               ; Subtract: DeltaTSC.High = EndTSC.High - StartTSC.High - Borrow
test edx,edx
jnz TimerFailed           ; This debug 32-bit code not supports > 4GHz
;--- Exit points ---
add esp,32
clc
TimerDone:
pop ebp esi edi
ret
TimerFailed:
add esp,32
stc
jmp TimerDone
