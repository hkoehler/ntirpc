/*
 * Copyright (c) 2015, PernixData, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Sun Microsystems, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TIRPC_RPC_TRACER_H
#define _TIRPC_RPC_TRACER_H

#include <rpc/types.h>     /* some typedefs */

#define TIRPC_TRACER   /* XXX */

/* List of trace points and their context pointer type */
typedef enum
{
   TIRPC_TRACE_EPOLL = 0,                    /* uint32_t channel ID */
   TIRPC_TRACE_EPOLL_EVENT,                  /* uint32_t channel ID */
   TIRPC_TRACE_GETREQ_ENTER,                 /* thread-specific pointer */
   TIRPC_TRACE_GETREQ_EXIT,                  /* thread-specific pointer */
   TIRPC_TRACE_SVC_VC_READAHEAD_ENTER,       /* SVCXPRT export */
   TIRPC_TRACE_SVC_VC_READAHEAD_EXIT,        /* SVCXPRT export */
   TIRPC_TRACE_SVC_VC_REQ_EVENT,             /* SVCXPRT export */
   TIRPC_TRACE_SVC_IOQ_ENTER,                /* xdr_ioq */
   TIRPC_TRACE_SVC_IOQ_EXIT,                 /* xdr_ioq */
   TIRPC_TRACE_SVC_IOQ_FLUSH_ENTER,          /* xdr_ioq */
   TIRPC_TRACE_SVC_IOQ_FLUSH_EXIT,           /* xdr_ioq */
   TIRPC_TRACE_RPC_CALL,                     /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_XFER_REPLY,          /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_REPLY_READY_ENTER,   /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_REPLY_READY_EXIT,    /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_WAIT_REPLY_ENTER,    /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_WAIT_REPLY_EXIT,     /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_WAIT_REPLY_TIMEOUT,  /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_DECODE_REPLY_ENTER,  /* rpc_ctx_t */
   TIRPC_TRACE_RPC_CALL_DECODE_REPLY_EXIT,   /* rpc_ctx_t */
   TIRPC_TRACE_MAX
} TIRPC_TRACE_POINTS;

#ifdef TIRPC_TRACER
#define __tracex(trace_point, ctx)                                \
   do {                                                           \
      if (__pkg_params.tracex) {                                  \
         __pkg_params.tracex(trace_point, (void *) ctx);          \
      }                                                           \
   } while (0)
#else
#define __tracex(trace_point, ptr, type)
#endif

#endif
