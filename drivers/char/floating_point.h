/* files from: Linux/arch/s390/kernel/floatlib.c */


#define SIGNBIT         0x80000000
#define EXCESSD         1022
#define HIDDEND         (1 << 20)
#define EXPD(fp)        (((fp.l.upper) >> 20) & 0x7FF)
#define SIGND(fp)       ((fp.l.upper) & SIGNBIT)
#define MANTD(fp)       (((((fp.l.upper) & 0xFFFFF) | HIDDEND) << 10) |(fp.l.lower >> 22))
#define HIDDEND_LL      ((long long)1 << 52)
#define MANTD_LL(fp)    ((fp.ll & (HIDDEND_LL-1)) | HIDDEND_LL)
#define PACKD_LL(s,e,m) (((long long)((s)+((e)<<20))<<32)|(m))


union double_long {
    double d;
#ifdef SWAP
    struct {
      unsigned long lower;
      long upper;
    } l;
#else
    struct {
      long upper;
      unsigned long lower;
    } l;
#endif
    long long ll;
};

int __eqdf2 (double a1, double a2)
{
     return *(long long *) &a1 == *(long long *) &a2;
}
int __nedf2 (double a1, double a2)
{
    return *(long long *) &a1 != *(long long *) &a2;
}

 /* addtwo doubles */
double __adddf3 (double a1, double a2)
{
    register long long mant1, mant2;
    register union double_long fl1, fl2;
    register int exp1, exp2;
    int sign = 0;

    fl1.d = a1;
    fl2.d = a2;

    /* check for zero args */
    if (!fl2.ll)
        goto test_done;
    if (!fl1.ll) {
        fl1.d = fl2.d;
        goto test_done;
    }

    exp1 = EXPD(fl1);
    exp2 = EXPD(fl2);

    if (exp1 > exp2 + 54)
        goto test_done;
    if (exp2 > exp1 + 54) {
        fl1.d = fl2.d;
         goto test_done;
    }

    /* do everything in excess precision so's we can round later */
    mant1 = MANTD_LL(fl1) << 9;
    mant2 = MANTD_LL(fl2) << 9;

    if (SIGND(fl1))
        mant1 = -mant1;
    if (SIGND(fl2))
        mant2 = -mant2;

    if (exp1 > exp2)
        mant2 >>= exp1 - exp2;
    else {
        mant1 >>= exp2 - exp1;
        exp1 = exp2;
    }
    mant1 += mant2;

    if (mant1 < 0) {
        mant1 = -mant1;
        sign = SIGNBIT;
    } else if (!mant1) {
        fl1.d = 0;
        goto test_done;
    }

    /* normalize up */
    while (!(mant1 & ((long long)7<<61))) {
        mant1 <<= 1;
        exp1--;
    }

    /* normalize down? */
    if (mant1 & ((long long)3<<62)) {
        mant1 >>= 1;
        exp1++;
    }

    /* round to even */
    mant1 += (mant1 & (1<<9)) ? (1<<8) : ((1<<8)-1);

    /* normalize down? */
    if (mant1 & ((long long)3<<62)) {
        mant1 >>= 1;
        exp1++;
    }

    /* lose extra precision */
    mant1 >>= 9;

    /* turn off hidden bit */
    mant1 &= ~HIDDEND_LL;

    /* pack up and go home */
    fl1.ll = PACKD_LL(sign,exp1,mant1);

test_done:
    return (fl1.d);
}



/* divide two doubles */
double __divdf3 (double a1, double a2)
{
    register union double_long fl1, fl2;
    register long long mask,result;
    register int exp, sign;

    fl1.d = a1;
    fl2.d = a2;
 
    /* subtract exponents */
    exp = EXPD(fl1) - EXPD(fl2) + EXCESSD;

    /* compute sign */
    sign = SIGND(fl1) ^ SIGND(fl2);

    /* numerator zero??? */
    if (fl1.ll == 0) {
        /* divide by zero??? */
        if (fl2.ll == 0)
            fl1.ll = ((unsigned long long)1<<63)-1;     /* NaN */
        else
            fl1.ll = 0;
        goto test_done;
    }

    /* return +Inf or -Inf */
    if (fl2.ll == 0) {
        fl1.ll = PACKD_LL(SIGND(fl1),2047,0);
         goto test_done;
    }


    /* now get mantissas */
    fl1.ll = MANTD_LL(fl1);
    fl2.ll = MANTD_LL(fl2);

     /* this assures we have 54 bits of precision in the end */
    if (fl1.ll < fl2.ll) {
        fl1.ll <<= 1;
        exp--;
    }

    /* now we perform repeated subtraction of fl2.ll from fl1.ll */
    mask = (long long)1<<53;
    result = 0;
    while (mask) {
        if (fl1.ll >= fl2.ll)
        {
            result |= mask;
            fl1.ll -= fl2.ll;
        }
        fl1.ll <<= 1;
        mask >>= 1;
    }

    /* round */
    result += 1;

    /* normalize down */
    exp++;
    result >>= 1;

    result &= ~HIDDEND_LL;

    /* pack up and go home */
    fl1.ll = PACKD_LL(sign, exp, result);

test_done:
    return (fl1.d);
}

/* subtract two doubles */
double __subdf3 (double a1, double a2)
{
    register union double_long fl1, fl2;

    fl1.d = a1;
    fl2.d = a2;

    /* check for zero args */
    if (!fl2.ll)
        return (fl1.d);
    /* twiddle sign bit and add */
    fl2.l.upper ^= SIGNBIT;
    if (!fl1.ll)
        return (fl2.d);
    return __adddf3 (a1, fl2.d);
}

/* multiply two doubles */
double __muldf3 (double a1, double a2)
{
    register union double_long fl1, fl2;
    register unsigned long long result=0ULL;
    register int exp;
    int sign;

    fl1.d = a1;
    fl2.d = a2;

    if (!fl1.ll || !fl2.ll) {
        fl1.d = 0;
        goto test_done;
    }

    /* compute sign and exponent */
    sign = SIGND(fl1) ^ SIGND(fl2);
    exp = EXPD(fl1) - EXCESSD;
    exp += EXPD(fl2);

    fl1.ll = MANTD_LL(fl1);
    fl2.ll = MANTD_LL(fl2);

    /* the multiply is done as one 31x31 multiply and two 31x21 multiples */
    result = (fl1.ll >> 21) * (fl2.ll >> 21);
    result += ((fl1.ll & 0x1FFFFF) * (fl2.ll >> 21)) >> 21;
    result += ((fl2.ll & 0x1FFFFF) * (fl1.ll >> 21)) >> 21;

    result >>= 2;
    if (result & ((long long)1<<61)) {
        /* round */
        result += 1<<8;
        result >>= 9;
    } else {
        /* round */
        result += 1<<7;
        result >>= 8;
        exp--;
    }
    if (result & (HIDDEND_LL<<1)) {
        result >>= 1;
        exp++;
    }

    result &= ~HIDDEND_LL;

    /* pack up and go home */
    fl1.ll = PACKD_LL(sign,exp,result);
test_done:
    return (fl1.d);
}
/* convert double to int */
long
__fixdfsi (double a1)
{
   register union double_long dl1;
  register int exp;
  register long l;

  dl1.d = a1;

  if (!dl1.l.upper && !dl1.l.lower)
    return (0);

  exp = EXPD (dl1) - EXCESSD - 31;
  l = MANTD (dl1);

  if (exp > 0)
      return SIGND(dl1) ? (1<<31) : ((1ul<<31)-1);

  /* shift down until exp = 0 or l = 0 */
  if (exp <= 0 && exp > -32 && l)
    l >>= -exp;
  else
    return (0);

  return (SIGND (dl1) ? -l : l);
}

double __floatsidf (register long a1)
{
  register int sign = 0, exp = 31 + EXCESSD;
  union double_long dl;

  if (a1 == 0x80000000)
    {
      /*
       * -a1 would be 0 !
       */
      dl.l.upper = 0xc1e00000;
      dl.l.lower = 0x0;
      return (dl.d);
    }

   if (!a1)
    {
      dl.l.upper = dl.l.lower = 0;
      return (dl.d);
    }

  if (a1 < 0)
    {
      sign = SIGNBIT;
      a1 = -a1;
    }

  while (a1 < 0x1000000)
    {
      a1 <<= 4;
      exp -= 4;
    }

  while (a1 < 0x40000000)
    {
      a1 <<= 1;
      exp--;
    }

  /* pack up and go home */
  dl.l.upper = sign;
  dl.l.upper |= exp << 20;
  dl.l.upper |= (a1 >> 10) & ~HIDDEND;
  dl.l.lower = a1 << 22;

  return (dl.d);
}
/* convert double to int */
long __floatunsidf (double a1)
{
  register union double_long dl1;
  register int exp;
  register long l;

  dl1.d = a1;

  if (!dl1.l.upper && !dl1.l.lower)
    return (0);

  exp = EXPD (dl1) - EXCESSD - 31;
  l = MANTD (dl1);

  if (exp > 0)
      return SIGND(dl1) ? (1<<31) : ((1ul<<31)-1);

  /* shift down until exp = 0 or l = 0 */
  if (exp <= 0 && exp > -32 && l)
    l >>= -exp;
  else
    return (0);

  return (SIGND (dl1) ? -l : l);
}



/*
 * floating point support end
 */





