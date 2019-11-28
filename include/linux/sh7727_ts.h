#ifndef SH7727_TS
#define SH7727_TS


#define emX(var) do{printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("%s=0x%08x\n",#var,(uint32_t)var);}while(0);

#endif
