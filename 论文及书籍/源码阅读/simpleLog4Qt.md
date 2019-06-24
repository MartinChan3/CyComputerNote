simpleLog4Qt是一个简单的log日志库

1. 版本写法：它使用了宏编写版本检查的方法，这个写法非常有意思，可以借鉴：   
```
#define SQTL_VERSION_STR   "1.2.0"
#define SQTL_VERSION       0x010200   // Version is: (major << 16) + (minor << 8) + patch
// SQTL_VERSION_CHECK can be used like: #if (SQTL_VERSION >= SQTL_VERSION_CHECK(1, 1, 0))
#define SQTL_VERSION_CHECK(major,minor,patch)   ((major<<16)|(minor<<8)|(patch))
```

2. ifdef和ifndef等同于if defined()和if !defined()

3. 在宏定义中，会广泛的使用\符号作为连接符，作为上下两行间过长的一个连接：   
```
#define SQTL_L_BODY(text,levelEnabledHard,levelEnabledSoft,level) \
  SQTL_MSVC_WARNING_SUPPRESS \
  do { if(levelEnabledHard && levelEnabledSoft) simpleqtlogger::SimpleQtLogger::getInstance()->log(text, level, __FUNCTION__, __FILE__, __LINE__); } while(0) \
  SQTL_MSVC_WARNING_RESTORE
```    

4. do while(0):宏定义中经常使用，合理规避有些情况需要加分号/不加分号或者有多个语句需要执行的情况，和上条结合使用；   

5. 运行的的本质：封装log函数为宏
```
#define L_FATAL(text)   SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_FATAL,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_FATAL,simpleqtlogger::LogLevel_FATAL)
#define L_ERROR(text)   SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_ERROR,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_ERROR,simpleqtlogger::LogLevel_ERROR)
#define L_WARN(text)    SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_WARNING,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_WARNING,simpleqtlogger::LogLevel_WARNING)
#define L_NOTE(text)    SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_NOTE,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_NOTE,simpleqtlogger::LogLevel_NOTE)
#define L_INFO(text)    SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_INFO,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_INFO,simpleqtlogger::LogLevel_INFO)
#define L_DEBUG(text)   SQTL_L_BODY(text,ENABLE_SQTL_LOG_LEVEL_DEBUG,simpleqtlogger::ENABLE_LOG_LEVELS.logLevel_DEBUG,simpleqtlogger::LogLevel_DEBUG)
```
log函数本质为发送对应信号：   
```
void SimpleQtLogger::log(const QString& text, LogLevel logLevel, const QString& functionName, const char* fileName, unsigned int lineNumber)
{
  // qDebug("SimpleQtLogger::log");

  // thread-safe

  emit signalLog(timeStamp(), threadId(), text, logLevel, functionName, fileName, lineNumber);
}
```   
注意此处特地强调**信号发送是线程绝对安全的**（非常讨巧的特性）。这意味着使用单纯的信号发送方式，得到的结果必然以postEvent()的方式在内存当中运行。        

6. __FUNCTION__、__FILE__、__LINE__分别代指当前代码函数、文件和行数，用于调试用   

7. 该库在使用命令行输出的时候，使用了基本的ESC_CODE的原理来对问题进行颜色输出;

8. 
