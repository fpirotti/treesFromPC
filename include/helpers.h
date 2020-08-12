#ifndef HELPERS_H
#define HELPERS_H


#include <laszip/laszip_api.h>



static void dll_error(laszip_POINTER laszip)
{
  if (laszip)
  {
    laszip_CHAR* error;
    if (laszip_get_error(laszip, &error))
    {
      fprintf(stderr,"DLL ERROR: getting error messages\n");
    }
    fprintf(stderr,"DLL ERROR MESSAGE: %s\n", error);
  }
}

#endif // HELPERS_H
