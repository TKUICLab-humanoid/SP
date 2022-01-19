/*****************************************************************************/
/*! \file    utilities.cpp
 *  \author  LH
 *  \date    2011-05-30
 *
 *  \brief   Miscellaneous functions
 *****************************************************************************/
#include "tku_libs/robocup_referee/utilities.h"

namespace robocup_referee
{
int chars_to_int(char const* chaine, int debut, int fin)
{
  int somme = 0;
  int i = 0;
  int mult = 1;
  while (debut + i < fin)
  {
    if (debut + i < fin - 1 && chaine[debut + i] < 0)
      somme += mult * (256 + chaine[debut + i]);
    else
      somme += chaine[debut + i] * mult;
    i++;
    mult *= 256;
  }
  return somme;
}

unsigned int chars_to_unsigned_int(char const* chaine, int debut, int fin)
{
  unsigned int somme = 0;
  int i = 0;
  unsigned int mult = 1;
  while (debut + i < fin)
  {
    somme += mult * (unsigned int)chaine[debut + i];
    i++;
    mult *= 256;
  }
  return somme;
}

void add_int_to_chain(int nb, char* chaine, int debut, int fin)
{
  if (nb < 0)
  {
    int mult = 256;
    for (int i = debut + 1; i < fin; i++)
    {
      nb -= mult;
      mult *= 256;
    }
  }
  while (debut < fin)
  {
    chaine[debut] = (char)nb;
    nb /= 256;
    debut++;
  }
}

}  // namespace robocup_referee
