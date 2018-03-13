/*

Linux memory (Cache and RAM) benchmark.

Options list (can be extended later):

operation = memory operation, values: read, write, copy
method = select CPU instruction set, yet available: sse128 (ia32, x64), avx256 (for x64 only)
nontemporal = select data usage mode, values: 0 or 1
threads = select number of execution threads: numeric value
min = minimum block size, starts from this value: numeric value + K/M/G
max = maximum block size, stops at this value: numeric value + K/M/G
step = step for block size, adds this value: numeric value + K/M/G
pages = paging option, values: normal, huge
precision = time or precision priority, values: fast, slow
machinereadable = make output machine readable (hex data), values: 0 or 1

BUGS AND NOTES.
 - all delta time visual, for all 4 timers
 - multi thread
 - avx256, avx512, mmx, mov
 - temporal and non-temporal memory access
 + strings sequences some printf as one string
 - carefully variables declaration
 - see program options
 + exit codes: 0=ok, 1=command line parse, 2=platform detect
 - parameter-specific verification
 - output help text block if incorrect command line
 - huge page size must get by OS API, yet predefined constant used

*/


#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>

//--- Title string and architecture-specific constants ---
#ifdef __x86_64__
#define TITLE "Memory benchmark for Linux 64.\n(C)2018 IC Book Labs. v0.16"
#define BUFALIGN_HUGEPAGE 2048*1024   // alignment factor, 2MB is huge page size for x64
#else
#define TITLE "Memory benchmark for Linux 32.\n(C)2018 IC Book Labs. v0.16"
#define BUFALIGN_HUGEPAGE 4096*1024   // alignment factor, 4MB is huge page size for ia32
#endif

//--- Defaults definitions ---
#define NOT_SET -1           // constant means no overrides for option, set default or default=f(sys)
#define OPERATION 0          // operation default is read
#define METHOD_SSE_128 0     // processor instruction set default is SSE 128-bit if AVX not supported
#define METHOD_AVX_256 1     // processor instruction set default is AVX 256-bit if AVX supported
#define NONTEMPORAL 0        // non-temporal data mode disabled by default
#define THREADS 1            // number of execution threads
#define MIN 4096             // minimum size of default data block
#define MAX 65536            // maximum size of default data block
#define STEP 1024            // default step from min to max is 512 bytes
#define PAGE_MODE_DEFAULT 0  // default page mode, 0 means classic 4KB
#define PRECISION 0          // default fast test, not a precision test
#define MACHINEREADABLE 0    // machine readable output disabled by default

#define BUFALIGN 4096        // alignment factor, 4KB is page size for x86/x64

//--- Text data for interpreting command line options ---
#define n_op 3
static char* operations[] = { "read", "write", "copy" };
#define n_mt 2
static char* methods[] = { "sse128" , "avx256" };
#define n_pg 2
static char* pages[] = { "normal", "huge" };
#define n_pr 2
static char* precisions[] = { "fast", "slow" };

//--- Numeric data for storing command line options, with defaults assigned ---
static int operation  = OPERATION;
static int method = NOT_SET;
static int nontemporal = NONTEMPORAL;
static int threads = THREADS;
static size_t min = MIN;
static size_t max = MAX;
static size_t step = STEP;
static int pagemode = PAGE_MODE_DEFAULT;
static int precision = PRECISION;
static int machinereadable = MACHINEREADABLE;

//--- Numeric data for storing scan configuration results ---
static size_t    bufferAlign = BUFALIGN;
static size_t    bufferSize = 0;
static char*     srcBase = NULL;
static char*     dstBase = NULL;
static size_t    srcAllocSize = 0;
static size_t    dstAllocSize = 0; 
static long long cpuFeatures  = 0;
static long long tscFrequency = 0;

//--- List of Linux timers IDs and its names ---
#define TCNT 4
static int clk_ids[] =
    { 
    CLOCK_REALTIME ,
    CLOCK_MONOTONIC ,
    CLOCK_PROCESS_CPUTIME_ID ,  // this timer not counts when sleep
    CLOCK_THREAD_CPUTIME_ID     // this also
    };
static char nameT0[] = "CLOCK_REALTIME          ";
static char nameT1[] = "CLOCK_MONOTONIC         ";
static char nameT2[] = "CLOCK_PROCESS_CPUTIME_ID";
static char nameT3[] = "CLOCK_THREAD_CPUTIME_ID ";
static char* namesT[] = { nameT0, nameT1, nameT2, nameT3 };

// Variables for OS timers support
static struct timespec ts[TCNT];              // reports of timers parameters
static struct timespec ts1[TCNT], ts2[TCNT];  // start and end moments

//--- Control block for command line parse ---
typedef enum
    { INTPARM, MEMPARM, SELPARM } OPTION_TYPES;
typedef struct
    {
    char* name;             // pointer to parm. name for recognition NAME=VALUE
    char** values;          // pointer to array of strings pointers, text opt.
    int n_values;           // number of strings for text option recognition
    void* data;             // pointer to updated option variable
    OPTION_TYPES routine;   // select handling method for this entry
    } OPTION_ENTRY;
    
#define OPTION_COUNT 10     // number of entries for command line options
static OPTION_ENTRY option_list[] =
    {
        { "operation"       , operations , n_op , &operation       , SELPARM },
        { "method"          , methods    , n_mt , &method          , SELPARM },
        { "nontemporal"     , NULL       , 0    , &nontemporal     , INTPARM },
        { "threads"         , NULL       , 0    , &threads         , INTPARM },
        { "min"             , NULL       , 0    , &min             , MEMPARM },
        { "max"             , NULL       , 0    , &max             , MEMPARM },
        { "step"            , NULL       , 0    , &step            , MEMPARM },
        { "pages"           , pages      , n_pg , &pagemode        , SELPARM },
        { "precision"       , precisions , n_pr , &precision       , SELPARM },
        { "machinereadable" , NULL       , 0    , &machinereadable , INTPARM }
    };

//--- Control block for start conditions parameters visual ---
typedef enum
    { INTEGER, MEMSIZE, SELECTOR, POINTER, HEX64, MHZ } PRINT_TYPES;
typedef struct
    {
    char* name;             // pointer to parameter name for visual NAME=VALUE 
    char** values;          // pointer to array of strings pointers, text opt.
    void* data;             // pointer to visualized option variable
    PRINT_TYPES routine;    // select handling method for this entry
    } PRINT_ENTRY;

#define PRINT_COUNT 17    // number of entries for print
#define PRINT_NAME  22    // number of chars before "=" for tabulation
static PRINT_ENTRY print_list[] = 
    {
        { "Memory operation"      , operations , &operation       , SELECTOR },
        { "Method"                , methods    , &method          , SELECTOR },
        { "Non-temporal mode"     , NULL       , &nontemporal     , INTEGER  },
        { "Threads count"         , NULL       , &threads         , INTEGER  },
        { "Minimum block size"    , NULL       , &min             , MEMSIZE  },
        { "Maximum block size"    , NULL       , &max             , MEMSIZE  },
        { "Block size step"       , NULL       , &step            , MEMSIZE  },
        { "Paging advise"         , pages      , &pagemode        , SELECTOR },
        { "Precision option"      , precisions , &precision       , SELECTOR },
        { "Machine readable"      , NULL       , &machinereadable , INTEGER  },
        { "Source pointer"        , NULL       , &srcBase         , POINTER  },
        { "Destination pointer"   , NULL       , &dstBase         , POINTER  },
        { "Source allocated"      , NULL       , &srcAllocSize    , MEMSIZE  },
        { "Destination allocated" , NULL       , &dstAllocSize    , MEMSIZE  },
        { "Page alignment"        , NULL       , &bufferAlign     , MEMSIZE  },
        { "Methods bitmap"        , NULL       , &cpuFeatures     , HEX64    },
        { "TSC frequency"         , NULL       , &tscFrequency    , MHZ      }
    }; 

//--- Helper method for print memory size: bytes/KB/MB/GB, overloaded ---
#define KILO 1024
#define MEGA 1024*1024
#define GIGA 1024*1024*1024
#define PRINT_LIMIT 20
int scratchMemorySize( char* scratchPointer, size_t memsize )
    {
    double xd = memsize;
    int nchars = 0;
    if ( memsize < KILO )
        {
        int xi = memsize;
        nchars = snprintf( scratchPointer, PRINT_LIMIT, "%d bytes", xi );
        }
    else if ( memsize < MEGA )
        {
        xd /= KILO;
        nchars = snprintf( scratchPointer, PRINT_LIMIT, "%.2lfK", xd );
        }
    else if ( memsize < GIGA )
        {
        xd /= MEGA;
        nchars = snprintf( scratchPointer, PRINT_LIMIT, "%.2lfM", xd );
        }
    else
        {
        xd /= GIGA;
        nchars = snprintf( scratchPointer, PRINT_LIMIT, "%.2lfG", xd );
        }
    return nchars;
    }

//--- Helper method for print memory size: bytes/KB/MB/GB, overloaded ---
int printMemorySize( size_t memsize )
    {
    double xd = memsize;
    int nchars = 0;
    if ( memsize < KILO )
        {
        int xi = memsize;
        nchars = printf( "%d bytes", xi );
        }
    else if ( memsize < MEGA )
        {
        xd /= KILO;
        nchars = printf( "%.2lfK", xd );
        }
    else if ( memsize < GIGA )
        {
        xd /= MEGA;
        nchars = printf( "%.2lfM", xd );
        }
    else
        {
        xd /= GIGA;
        nchars = printf( "%.2lfG", xd );
        }
    return nchars;
    }

//--- Helper method for print selected string from strings array ---
void printSelectedString( int select, char* names[] )
    {
    printf( "%s", names[select] );
    }

//--- Detect, store and print OS timers configuration ---
void detectAndPrintTimers()
{
int i = 0;
int clockStatus = 0;
unsigned long long int seconds = 0, nanoseconds = 0;
for ( i=0; i<TCNT; i++ )
    {
    clockStatus = clock_getres(clk_ids[i], &ts[i] );
    if( clockStatus==0 )
        {
        seconds      = ts[i].tv_sec;
        nanoseconds  = ts[i].tv_nsec;
        double xs = seconds;
        double xn = nanoseconds;
        printf( "%s  %.lf s %.lf ns\n", namesT[i], xs, xn );
        }
    else
        {
        ts[i].tv_sec = -1;
        ts[i].tv_nsec = -1;
        printf( "%s  N/A ( %s )\n", namesT[i], strerror(errno) );
        }
    }
}

//--- Called at start of measured interval ---
void startTimeDelta()
{
int i = 0;
int timerStatus = 0;
for ( i=0; i<TCNT; i++ )
    {
    if ( ts[i].tv_sec >= 0 )  // validation from get units
        {  // get current time, return status, it checked
        timerStatus = clock_gettime(clk_ids[i], &ts1[i] );
        }
    else
        {  // set error condition if previous get time units failed
        timerStatus = -1;
        }
    if( timerStatus!=0 )  // validation from get time result
        {
        ts1[i].tv_sec = -1;   // force invalid result
        ts1[i].tv_nsec = -1;
        }
    }
}

//--- Called at end of measured interval ---
void stopTimeDelta()
{
int i = 0;
int timerStatus = 0;
for ( i=0; i<TCNT; i++ )
    {
    if ( ts1[i].tv_sec >= 0 )  // validation from get start time
        {  // get current time, return status, it checked
        timerStatus = clock_gettime(clk_ids[i], &ts2[i] );
        }
    else
        {  // set error condition if previous get start time failed
        timerStatus = -1;
        }
    if( timerStatus!=0 )      // validation from get time result
        {
        ts2[i].tv_sec = -1;   // force invalid result
        ts2[i].tv_nsec = -1;
        }
    }
}

//--- Get and print Linux application statistics ---
void printStatistics()
{
struct rusage usage;
int usageStatus = 0;
int why = RUSAGE_SELF;
usageStatus = getrusage ( why, &usage );
if ( usageStatus < 0 )
    {
    printf ( "Get resource usage failed ( %s )\n" , strerror(errno) );
    exit(2);
    }
printf ( "User space CPU time used: %ld sec %ld usec\n", 
         (long)(usage.ru_utime.tv_sec), (long)(usage.ru_utime.tv_usec) );
printf ( "System space CPU time used: %ld sec %ld usec\n", 
         (long)(usage.ru_stime.tv_sec), (long)(usage.ru_stime.tv_usec) );
printf ( "Maximum resident set size        = %ld KB\n", usage.ru_maxrss );
printf ( "Integral shared memory size      = %ld KB\n", usage.ru_ixrss );
printf ( "Integral unshared data size      = %ld KB\n", usage.ru_idrss );
printf ( "Integral unshared stack size     = %ld KB\n", usage.ru_isrss );
printf ( "Page reclaims (soft page faults) = %ld\n", usage.ru_minflt );
printf ( "Page faults (hard page faults)   = %ld\n", usage.ru_majflt );
printf ( "Swaps                            = %ld\n", usage.ru_nswap );
printf ( "Block input operations           = %ld\n", usage.ru_inblock );
printf ( "Block output operations          = %ld\n", usage.ru_oublock );
printf ( "IPC messages sent                = %ld\n", usage.ru_msgsnd );
printf ( "IPC messages received            = %ld\n", usage.ru_msgrcv );
printf ( "Signals received                 = %ld\n", usage.ru_nsignals );
printf ( "Voluntary context switches       = %ld\n", usage.ru_nvcsw );
printf ( "Involuntary context switches     = %ld\n", usage.ru_nivcsw );
}

//--- Names of tests ---
static char* testsNames[] = 
    { "Read SSE 128bit" , "Write SSE 128bit" , "Copy SSE 128bit" ,
      "Read AVX 256bit" , "Write AVX 256bit" , "Copy AVX 256bit" };

//--- Values of bytes per instruction for convert instructions to megabytes ---
static int bytesPerInstruction[] = 
    { 16 , 16 , 16 ,
      32 , 32 , 32 };

//--- Routones for CPU and Platform features detect ---
// parm#1 = pointer for return qword: features bitmap, yet used bit[0] = sse128
// parm#2 = pointer for return qword: TSC clock frequency, Hz
// parm#3 = reserved for return qword: CPU context management bitmap
// parm#4 = reserved for return qword: OS context management bitmap
// return reserved
int GetCPUinfo ( long long*, long long* );

//--- Routines for tests ---
// parm #1 = pointer for return number of TSC clocks
// parm #2 = pointer for return number of instructions
// parm #3 = pointer to target memory array for read/write benchmarking
// parm #4 = size of target memory array, bytes
// parm #5 = pointer to second buffer, valid for copy only
// return reserved
int ReadSSE128  ( long long*, long long*, char[], int, char[] );
int WriteSSE128 ( long long*, long long*, char[], int, char[] );
int CopySSE128  ( long long*, long long*, char[], int, char[] );
int ReadAVX256  ( long long*, long long*, char[], int, char[] );
int WriteAVX256 ( long long*, long long*, char[], int, char[] );
int CopyAVX256  ( long long*, long long*, char[], int, char[] );

int( *asmRoutines[6] ) 
( long long*, long long*, char[], int, char[] ) =
{ ReadSSE128 , WriteSSE128 , CopySSE128 ,
  ReadAVX256 , WriteAVX256 , CopyAVX256 };

//---------- Application entry point -------------------------------------------

int main( int argc, char** argv )
{
//--- Start message ---
printf ( "\n%s\n\n", TITLE );

//--- Initializing variables ---
int i=0, j=0, k=0, k1=0, k2=0;  // miscellaneous counters and variables
int recognized = 0;             // result of strings comparision, 0=match 
OPTION_TYPES t = 0;             // enumeration of parameters types for accept
char* pAll = NULL;              // pointer to option full string NAME=VALUE
char* pName = NULL;             // pointer to sub-string NAME
char* pValue = NULL;            // pointer to sub-string VALUE
char* pPattern = NULL;          // pointer to compared pattern string
char** pPatterns = NULL;        // pointer to array of pointers to pat. strings
int* pInt = NULL;               // pointer to integer (32b) for variable store
size_t* pPointer = NULL;        // pointer to pointer (32/64b) for variable store
size_t kPointer;                // transit variable for memory block size
char c = 0;                     // transit storage for char
#define SMIN 3                  // minimum option string length, example a=b
#define SMAX 81                 // maximum option string length
char cmdName[SMAX];             // extracted NAME of option string
char cmdValue[SMAX];            // extracted VALUE of option string

//--- Accept command line options ---
for ( i=1; i<argc; i++ )        // cycle for command line options
    {
    // initializing for parsing current string
    // because element [0] is application name, starts from [1]
    pAll = argv[i];
    for ( j=0; j<SMAX; j++ )  // clear buffers
        {
        cmdName[j]=0;
        cmdValue[j]=0;
        }
    // check option sub-string length
    k = strlen(pAll);                   // k = length of one option sub-string
    if ( k<SMIN )
        {
        printf( "ERROR, OPTION TOO SHORT: %s\n", pAll );
        exit(1);
        }
    if ( k>SMAX )
        {
        printf( "ERROR, OPTION TOO LONG: %s\n", pAll );
        exit(1);
        }
    // extract option name and option value substrings
    pName = cmdName;
    pValue = cmdValue;
    strcpy( pName, pAll );           // store option sub-string to pName
    strtok( pName, "=" );            // pName = pointer to fragment before "="
    pValue = strtok( NULL, "?" );    // pValue = pointer to fragment after "="
    // check option name and option value substrings
    k1 = 0;
    k2 = 0;
    if ( pName  != NULL ) { k1 = strlen( pName );  }
    if ( pValue != NULL ) { k2 = strlen( pValue ); }
    if ( ( k1==0 )||( k2==0 ) )
        {
        printf( "ERROR, OPTION INVALID: %s\n", pAll );
        exit(1);
        }
    // detect option by comparision from list, cycle for supported options
    for ( j=0; j<OPTION_COUNT; j++ )
        {
        pPattern = option_list[j].name;
        recognized = strcmp ( pName, pPattern );
        if ( recognized==0 )
            {
            // option-type specific handling, run if name match
            t = option_list[j].routine;
            switch(t)
                {
                case INTPARM:  // support integer parameters
                    {
                    k1 = strlen( pValue );
                    for ( k=0; k<k1; k++ )
                        {
                        if ( isdigit( pValue[k] ) == 0 )
                            {
                            printf( "ERROR, NOT A NUMBER: %s\n", pValue );
                            exit(1);
                            }
                        }
                    k = atoi( pValue );   // convert string to integer
                    pInt = option_list[j].data;
                    *pInt = k;
                    break;
                    }
                case MEMPARM:  // support memory block size parameters
                    {
                    k1 = 0;
                    k2 = strlen( pValue );
                    c = pValue[k2-1];
                    if ( isdigit(c) != 0 )
                        {
                        k1 = 1;             // no units kilo, mega, giga
                        }
                    else if ( c == 'K' )    // K means kilobytes
                        {
                        k2--;               // last char not a digit K/M/G
                        k1 = 1024;
                        }
                    else if ( c == 'M' )    // M means megabytes
                        {
                        k2--;
                        k1 = 1024*1024;
                        }
                    else if ( c == 'G' )    // G means gigabytes
                        {
                        k2--;
                        k1 = 1024*1024*1024;
                        }
                    for ( k=0; k<k2; k++ )
                        {
                        if ( isdigit( pValue[k] ) == 0 )
                            {
                            k1 = 0;
                            }
                        }
                    if ( k1==0 )
                        {
                        printf( "ERROR, NOT A BLOCK SIZE: %s\n", pValue );
                        exit(1);
                        }
                    k = atoi( pValue );   // convert string to integer
                    kPointer = k;
                    kPointer *= k1;
                    pPointer = option_list[j].data;
                    *pPointer = kPointer;
                    break;
                    }
                case SELPARM:    // support parameters selected from text names
                    {
                    k1 = option_list[j].n_values;
                    k2 = 0;
                    pPatterns = option_list[j].values;
                    for ( k=0; k<k1; k++ )
                        {
                        pPattern = pPatterns[k];
                        k2 = strcmp ( pValue, pPattern );
                        if ( k2==0 )
                            {
                            pInt = option_list[j].data;
                            *pInt = k;
                            break;
                            }
                        }
                    if ( k2 != 0 )
                        {
                        printf( "ERROR, VALUE INVALID: %s\n", pAll );
                        exit(1);
                        }
                    break;
                    }
                }
            break;
            }
        }
    // check option name recognized or not
    if ( recognized != 0 )
        {
        printf( "ERROR, OPTION NOT RECOGNIZED: %s\n", pName );
        exit(1);
        }
    }

//--- Get system info ---

//--- Detect OS timers ---
printf( "OS timers list with resolutions:\n" );
detectAndPrintTimers();

//--- Get CPU features and measure TSC clock ---
printf ( "\nGet CPU info...\n" );
GetCPUinfo( &cpuFeatures, &tscFrequency );
if ( cpuFeatures == 0 )
    {
    printf( "%s\n", "Minimum CPU feature set missing" );
    exit(1);
    }
if ( ( ( cpuFeatures & 0x2 ) != 0 ) && ( method == NOT_SET )  )
    {
    method = METHOD_AVX_256;
    }
else if ( method == NOT_SET )
    {
    method = METHOD_SSE_128;
    }

//--- Allocate memory ---
printf ( "Allocate memory...\n" );

bufferSize = max;
srcAllocSize = bufferSize;
dstAllocSize = bufferSize;

if ( pagemode != 0 )
    {
    bufferAlign = BUFALIGN_HUGEPAGE;

    long long unsigned int bs = bufferSize;
    long long unsigned int ba = BUFALIGN_HUGEPAGE;
    long long unsigned int bsmodba = bs % ba;
    if ( bsmodba != 0 )
        {
        long long unsigned int allocbuf = ( bs / ba * ba ) + ba;
        srcAllocSize = allocbuf;
        dstAllocSize = allocbuf;
        }
    }

srcBase = memalign ( bufferAlign, srcAllocSize );
dstBase = memalign ( bufferAlign, dstAllocSize );
if ( (srcBase <= 0) || (dstBase <= 0) )
    {
    printf( "%s ( %s )\n", "Memory allocation failed", strerror(errno) );
    exit(1);
    }

//--- Advise buffer status ---
if ( pagemode != 0 )
    {
    int x1 = madvise ( srcBase, srcAllocSize, MADV_HUGEPAGE );
    int x2 = madvise ( dstBase, dstAllocSize, MADV_HUGEPAGE );
    if ( (x1 != 0 ) || ( x2 != 0 ) )
        {
        printf( "%s ( %s )\n", "Memory advise failed", strerror(errno) );
        exit(1);
        }
    }

//--- Clear memory, page faults better outside measured interval ---
// This code must be near to benchmarking because cache effects
for ( i=0; i<srcAllocSize; i++ )
     {
     srcBase[i] = 0;
     }
for ( i=0; i<dstAllocSize; i++ )
     {
     dstBase[i] = 0;
     }

//--- Print start conditions of test ---
printf ( "\nStart conditions:\n" );
// correct this: declarations irregular
PRINT_TYPES m = 0;            // enumeration of parameters types for print
long long unsigned int n = 0;         // value for block size
long long unsigned int* np = NULL;    // pointer to block size value
double d = 0.0;                  // transit variable
unsigned long long* dp = NULL;   // transit pointer to double
int*  kp = NULL;                 // pointer to integer
char* cp = NULL;                 // pointer to char
char** ccp = NULL;               // pointer to array of pointers to strings
size_t* sizep = 0;               // pointer to block size variable
size_t size = 0;                 // block size variable
// correct this: declarations irregular
for ( i=0; i<PRINT_COUNT; i++ )
    {
    k = PRINT_NAME - printf( "%s", print_list[i].name );
    for ( j=0; j<k; j++ )
        {
        printf(" ");
        }
    printf("= ");
    m = print_list[i].routine;
    switch(m)
        {
        case INTEGER:  // integer parameter
            {
            kp = print_list[i].data;
            k = *kp;
            printf( "%d", k );
            break;
            }
        case MEMSIZE:  // memory block size parameter
            {
            sizep = print_list[i].data;
            size = *sizep;
            printMemorySize( size );
            break;
            }
        case SELECTOR:  // pool of text names parameter
            {
            kp = print_list[i].data;
            k = *kp;
            ccp = print_list[i].values;
            printSelectedString( k, ccp );
            break;
            }
        case POINTER:  // memory pointer parameter
            {
            ccp = print_list[i].data;
            cp = *ccp;
            printf( "%p", cp );
            break;
            }
        case HEX64:  // 64-bit hex number parameter
            {
            np = print_list[i].data;
            n = *np;
            printf( "0x%08llX", n );
            break;
            }
        case MHZ:  // frequency in MHz parameter
            {
            dp = print_list[i].data;
            d = *dp;  // with convert from unsigned long long to double
            d /= 1000000.0;
            printf( "%.1f MHz", d );
            }
        }
    printf("\n");
    }

//--- Check start parameters validity and compatibility ---
if ( ( ( cpuFeatures & 0x2 ) == 0 ) && ( method == METHOD_AVX_256 )  )
    {
    printf( "\nPLATFORM ERROR: AVX not supported or locked.\n" );
    exit(1);
    }
if ( nontemporal != 0 )
    {
    printf("\nBAD PARAMETER: non-temporal mode yet not supported.\n");
    exit(1);
    }
    
if ( threads == 0 )
    {
    printf("\nBAD PARAMETER: zero numbers of threads.\n");
    exit(1);
    }
    
if ( threads != 1 )
    {
    printf("\nBAD PARAMETER: multi-thread not supported yet.\n");
    exit(1);
    }
    
long long limitmax = 1024*1024*1024;
long long limitmin = 256;
if ( ( min > limitmax )||( max > limitmax )||( step > limitmax )||
   ( min < limitmin )||( max < limitmin )||( step < limitmin ) )
    {
    printf("\nBAD PARAMETER: min,max,step must be 256 bytes ... 1GB.\n");
    exit(1);
    }

if ( precision != 0 )
    {
    printf("\nBAD PARAMETER: precision control not supported yet.\n");
    exit(1);
    }
    
if ( machinereadable != 0 )
    {
    printf("\nBAD PARAMETER: machine readable output not supported yet.\n");
    exit(1);
    }

//--- Wait for key (Y/N) with list of start parameters ---
printf("\nStart? (Y/N)" );
int key = 0;
key = getchar();
key = tolower(key);
if ( key != 'y' )
    {
    printf("Test skipped.\n");
    exit(3);
    }

//--- Target benchmark operation with time measurement, print results ---
printf( "\nBenchmarking (%s)...\n" , testsNames[ operation + method*3 ] );
printf( "\n Size       MBPS        Utilization  TSC/instr  ns/instr" );
printf( "\n-------------------------------------------------------------\n" );

size_t varSize = 0;                          // size, modified in cycle
long long deltaTSC = 0;                      // number of TSC clocks from asm
long long instructions = 0;                  // number of instructions from asm
int status = 0;                              // API functions status
double megabytes = 0.0;                      // block size calculation
double seconds = 0.0, nanoseconds = 0.0;     // time calculation
double mbps = 0.0;                           // megabytes per second
double timeTotal = 0.0, timeUtilized = 0.0;  // total and utilized time
double utilization = 0.0;                    // processor utilization
double fdtsc = 0.0, finstr = 0.0;            // dtsc and instr as double
double tscInstr = 0.0;                       // TSC clocks per one instruction
double nsInstr = 0.0;                        // nanoseconds per one instruction

// cycle for different sizes of tested block

#define MAXLINE  200            // maximum line size, chars, include last zero
#define MAXENTRY 20             // maximum sub-line size for parameter
char  scratchLine[MAXLINE];     // scratch buffer for line
char* scratchPointer = NULL;    // pointer for scratch buffer addressing
size_t spaces = 0;              // calculated for tabulations

for ( varSize=min; varSize<=max; varSize+=step )
    {
    // blank scratch line, initialize pointer
    for ( i=0; i<MAXLINE; i++ ) { scratchLine[i] = 0; }
    scratchPointer = scratchLine;
    // print size, better one string built with all this entries 
    spaces = snprintf( scratchPointer, 2, " " ); 
    scratchPointer += spaces;
    spaces = scratchMemorySize( scratchPointer, varSize ); 
    scratchPointer += spaces;
    // benchmarking
    startTimeDelta();
    status = asmRoutines[ operation + method*3 ]
        ( &deltaTSC, &instructions, srcBase, varSize, dstBase );
    stopTimeDelta();
    // calculate megabytes per second
    seconds = ts2[2].tv_sec - ts1[2].tv_sec;
    nanoseconds = ts2[2].tv_nsec - ts1[2].tv_nsec;
    seconds += nanoseconds / 1000000000.0;        // add nanoseconds
    megabytes = instructions * bytesPerInstruction[ operation + method*3 ];
    megabytes /= 1048576.0;
    mbps = megabytes / seconds;
    // calculate CPU utilization
    timeUtilized = seconds;
    seconds = ts2[0].tv_sec - ts1[0].tv_sec;
    nanoseconds = ts2[0].tv_nsec - ts1[0].tv_nsec;
    seconds += nanoseconds / 1000000000.0;        // add nanoseconds
    timeTotal = seconds;
    utilization = timeUtilized / timeTotal;
    // calculate TSC clocks per one instruction
    fdtsc = deltaTSC;
    finstr = instructions;
    tscInstr = fdtsc / finstr;
    // calculate nanoseconds per one instruction
    timeTotal *= 1000000000.0;     // convert to nanoseconds
    nsInstr = timeTotal / finstr;
    // print megabytes per second
    spaces = 11 - spaces;
    for (i=0; i<spaces; i++ )
        { scratchPointer += snprintf( scratchPointer, 2, " " ); }
    spaces = snprintf( scratchPointer, MAXENTRY, "%.2f", mbps );
    scratchPointer += spaces;
    // print CPU utilization
    spaces = 12 - spaces;
    for (i=0; i<spaces; i++ ) 
        { scratchPointer += snprintf( scratchPointer, 2, " " ); }
    spaces = snprintf( scratchPointer, MAXENTRY, "%.3f", utilization ); 
    scratchPointer += spaces;
    // print TSC clocks per one instruction
    spaces = 13 - spaces;
    for (i=0; i<spaces; i++ ) 
        { scratchPointer += snprintf( scratchPointer, 2, " " ); }
    spaces = snprintf( scratchPointer, MAXENTRY, "%.3f", tscInstr ); 
    scratchPointer += spaces;
    // print nanoseconds per one instruction
    spaces = 11 - spaces;
    for ( i=0; i<spaces; i++ ) 
        { scratchPointer += snprintf( scratchPointer, 2, " " ); }
    spaces = snprintf( scratchPointer, MAXENTRY, "%.3f", nsInstr ); 
    scratchPointer += spaces;
    // output one current line to console
    printf( "%s\n", scratchLine );
    }

printf( "-------------------------------------------------------------\n" );

//--- Release allocated memory ---
printf ( "\nRelease memory...\n" );
free( srcBase );
free( dstBase );

//--- Application statistics ---
//--- Print application statistics by OS info ---
printf ( "\nApplication statistics:\n" );
printStatistics();

//--- Done ---
// printf("\n");
exit(0);
}

