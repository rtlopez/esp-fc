#ifndef _ESP_TIMER_H_
#define _ESP_TIMER_H_

class EspTimer
{
  public:
    typedef void (*callback_ptr)(void*);

    void execute() const ICACHE_RAM_ATTR
    {
      if(_fn) _fn(_arg);
    }

  protected:
    callback_ptr _fn;
    void * _arg;
};

#endif