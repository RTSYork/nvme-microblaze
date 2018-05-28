/**
 * Copyright (c) 2015-2016, Micron Technology, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief Logging support routines.
 */

#include "../unvme/unvme_log.h"

#include <stdarg.h>
#include <pthread.h>



// Static global variables
static FILE*                log_fp = NULL;  ///< log file pointer
static int                  log_count = 0;  ///< log open count
//static pthread_mutex_t      log_lock = PTHREAD_MUTEX_INITIALIZER; ///< log lock


/**
 * Open log file.  Only one log file is supported and thus only the first
 * call * will create the log file by its specified name.  Subsequent calls
 * will only be counted.
 * @param   name        log filename
 * @param   mode        open mode
 * @return  0 indicating 
 */
int log_open(const char* name, const char* mode)
{
//    pthread_mutex_lock(&log_lock);
    if (!log_fp) {
        log_fp = fopen(name, mode);
        if (!log_fp) {
            perror("log_open");
//            pthread_mutex_unlock(&log_lock);
            return -1;
        }
    }
    log_count++;
//    pthread_mutex_unlock(&log_lock);
    return 0;
}

int log_open_stdout()
{
//    pthread_mutex_lock(&log_lock);
    if (!log_fp) {
        log_fp = stdout;
        if (!log_fp) {
            perror("log_open");
//            pthread_mutex_unlock(&log_lock);
            return -1;
        }
    }
    log_count++;
//    pthread_mutex_unlock(&log_lock);
    return 0;
}

/**
 * Close the log file (only the last close will effectively close the file).
 */
void log_close()
{
//    pthread_mutex_lock(&log_lock);
    if (log_count > 0) {
        if ((--log_count == 0) && log_fp && log_fp != stdout) {
            fclose(log_fp);
            log_fp = NULL;
        }
    }
//    pthread_mutex_unlock(&log_lock);
}

/**
 * Write a formatted message to log file, if log file is opened.
 * If err flag is set then log also to stderr.
 * @param   ftee        additional file to print to
 * @param   fmt         formatted message
 */
void log_msg(FILE* ftee, const char* fmt, ...)
{
    va_list args;

//    pthread_mutex_lock(&log_lock);
    if (log_fp) {
        va_start(args, fmt);
        if (ftee) {
            char s[4096];
            vsnprintf(s, sizeof(s), fmt, args);
            fprintf(ftee, "%s", s);
            fflush(ftee);
            fprintf(log_fp, "%s", s);
            fflush(log_fp);
        } else {
            vfprintf(log_fp, fmt, args);
            fflush(log_fp);
        }
        va_end(args);
    } else {
        va_start(args, fmt);
        if (ftee) {
            vfprintf(ftee, fmt, args);
            fflush(ftee);
        } else {
            vfprintf(stdout, fmt, args);
            fflush(stdout);
        }
        va_end(args);
    }
//    pthread_mutex_unlock(&log_lock);
}

