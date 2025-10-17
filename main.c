#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {

  int n;
  scanf("%d", &n);

  int i = 1;
  while ((n / (6 * ((i * i / 2) + (i / 2))) + 1) > 0) {
    i++;
  }
  printf("%d", i);

  return 0;
}