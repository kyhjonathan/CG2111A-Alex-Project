// When an Arduino first starts after being freshly programmed
// there might be stray characters in the serial buffer.
// This function purges those characters.
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
  int x;
  int y;
} TData;

TData test;

int main() {
  test.x = 5;
  test.y=10;

  for(int i = 0; i < 200; i++) {
    char theSize = (char) sizeof(TData); // size in bytes
    printf("%c %d\n", theSize, theSize);
    printf("%p\n", (char *) &test);
    //Serial.write((char *) &test, sizeof(TData));

    test.x++;
  }
}
