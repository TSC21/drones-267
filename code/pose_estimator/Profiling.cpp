#include "Profiling.h"
#include <cstddef>

double readTimer( )
{
    static int initialized = 0;
    static struct timeval start;
    struct timeval end;
    if( !initialized )
    {
        gettimeofday( &start, NULL );
        initialized = 1;
    }
    gettimeofday( &end, NULL );
    return (end.tv_sec - start.tv_sec) + 1.0e-6 * (end.tv_usec - start.tv_usec);
}

double getDifferenceAndIncrement(double before, double* counters, int index) 
{
    double now = readTimer();
    if (counters) 
    {
    	double difference = now - before;
    	counters[index] += difference;	
    }
    return now;
}

double getDifferenceAndIncrement(double before, double* total) 
{
    double now = readTimer();
    double difference = now - before;
    (*total) += difference;
    return now;
}