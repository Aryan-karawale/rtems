/**
 *  @file
 *  @brief iswctype_l() API Conformance Test
 */

/*
 *  COPYRIGHT (c) 2018.
 *  Jacob Shin
 *
 *  Permission to use, copy, modify, and/or distribute this software
 *  for any purpose with or without fee is hereby granted.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 *  WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
 *  BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
 *  OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 *  WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
 *  ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <wctype.h>
#include <locale.h>

int test( void );

int test( void )
{
  int return_value;
  wctype_t charclass = wctype("alnum");
  locale_t loc = newlocale(LC_ALL_MASK, "C", (locale_t) 0);

  return_value = iswctype_l('c', charclass, loc);
  return (return_value != 0);
}