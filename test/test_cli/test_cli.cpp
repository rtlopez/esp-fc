#include <Connect/Cli.hpp>
#include <Stream/Printer.hpp>
#include <cstring>
#include <string>
#include <unity.h>

using Cli = Espfc::Connect::Cli;
using CliCmd = Espfc::CliCmd;
using Model = Espfc::Model;

class StreamMock : public Espfc::Stream::Writable
{
public:
  StreamMock(): _buffer("") {}
  size_t write(uint8_t c) override
  {
    _buffer += static_cast<char>(c);
    return 1;
  }
  size_t write(const uint8_t* buffer, size_t size) override
  {
    _buffer.append(reinterpret_cast<const char*>(buffer), size);
    return size;
  }
  int availableForWrite() override
  {
    return 0;
  }
  void flush() override
  {
    _buffer.clear();
  }
  bool isTxFifoEmpty() override
  {
    return true;
  }
  const char* c_str() const
  {
    return _buffer.c_str();
  }
  std::string str() const
  {
    return _buffer;
  }

private:
  std::string _buffer;
};

static StreamMock stream;
static Espfc::Stream::Printer printer{stream};

void setUp(void)
{
  stream.flush();
}

void tearDown(void)
{
  stream.flush();
}

void test_cli_init()
{
  Model model;
  Cli cli{model};
  TEST_ASSERT_FALSE(cli._active);
  TEST_ASSERT_FALSE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);
  TEST_ASSERT_NOT_NULL(cli._params);
}

void test_cli_enter_interactive()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  cli.process('h', cmd, printer);
  TEST_ASSERT_TRUE(cli._active);
  TEST_ASSERT_TRUE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);
}

void test_cli_enter_leave_non_interactive()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  cli.process(0x02, cmd, printer);
  TEST_ASSERT_TRUE(cli._active);
  TEST_ASSERT_FALSE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);

  cli.process(0x03, cmd, printer);
  TEST_ASSERT_FALSE(cli._active);
  TEST_ASSERT_FALSE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);

  auto result = stream.str();
  TEST_ASSERT_EQUAL(2, result.length());
  TEST_ASSERT_EQUAL(0x02, result[0]);
  TEST_ASSERT_EQUAL(0x03, result[1]);
}

void test_cli_configurator_handshake()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  cli.process('#', cmd, printer);
  TEST_ASSERT_TRUE(cli._active);
  TEST_ASSERT_TRUE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_TRUE(result.find("Entering CLI Mode") != std::string::npos);

  stream.flush();

  // CTRL-D (0x04) is used to exit CLI mode
  cli.process(0x04, cmd, printer);
  TEST_ASSERT_FALSE(cli._active);
  TEST_ASSERT_FALSE(cli._interactive);
  TEST_ASSERT_FALSE(cli._ignore);

  result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("leaving CLI mode"));
}

void test_cli_process_comment()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("command # comment"))
  {
    cli.process(c, cmd, printer);
  }

  TEST_ASSERT_EQUAL(8, cmd.index);
  TEST_ASSERT_EQUAL_STRING("command ", cmd.buff);
  TEST_ASSERT_EQUAL(0, cmd.args[cmd.index]);
  TEST_ASSERT_EQUAL(std::string::npos, std::string{cmd.buff}.find("# comment"));

  TEST_ASSERT_TRUE(cli._active);
  TEST_ASSERT_TRUE(cli._interactive);
  TEST_ASSERT_TRUE(cli._ignore);

  cli.process('\n', cmd, printer);

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("# command"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("unknown command: command"));
}

void test_cli_overflow()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (size_t i = 0; i < sizeof(cmd.buff) + 2; ++i)
  {
    cli.process('a', cmd, printer);
  }

  TEST_ASSERT_EQUAL(sizeof(cmd.buff) - 1, cmd.index);
  TEST_ASSERT_EQUAL_STRING(std::string(sizeof(cmd.buff) - 1, 'a').c_str(), cmd.buff);
  TEST_ASSERT_EQUAL(0, cmd.args[cmd.index]);
}

void test_cli_process_help()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("help"))
  {
    cli.process(c, cmd, printer);
  }

  TEST_ASSERT_EQUAL(4, cmd.index);
  TEST_ASSERT_EQUAL_STRING("help", cmd.buff);
  TEST_ASSERT_EQUAL(0, cmd.args[cmd.index]);

  cli.process('\n', cmd, printer);

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("# help"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("available commands"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("defaults"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("reboot"));
}

void test_cli_process_help_non_interactive()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("\02help\n\03"))
  {
    cli.process(c, cmd, printer);
  }

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_EQUAL(0x02, result[0]);
  TEST_ASSERT_EQUAL(0x03, result[result.length() - 1]);
  TEST_ASSERT_EQUAL(std::string::npos, result.find("# help"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("available commands"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("defaults"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("reboot"));
}

void test_cli_get_mixer_type()
{
  Model model;
  model.config.mixer.type = 0;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("get mixer_type\n"))
  {
    cli.process(c, cmd, printer);
  }

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("set mixer_type NONE"));
}

void test_cli_set_mixer_type()
{
  Model model;
  model.config.mixer.type = 0;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("set mixer_type QUADX\n"))
  {
    cli.process(c, cmd, printer);
  }

  TEST_ASSERT_EQUAL(Espfc::FC_MIXER_QUADX, model.config.mixer.type);
}

void test_cli_bf_get_mag_calibration()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("get mag_calibration\n"))
  {
    cli.process(c, cmd, printer);
  }

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("mag_calibration ="));
}

void test_cli_bf_sensor_hardware()
{
  Model model;
  Cli cli{model};
  CliCmd cmd;

  for (char c : std::string("sensor_hardware\n"))
  {
    cli.process(c, cmd, printer);
  }

  auto result = stream.str();
  TEST_ASSERT_NOT_EQUAL(0, result.length());
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("gyro: NONE,"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("acc: NONE,"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("baro: AUTO,"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, result.find("mag: AUTO,"));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_cli_init);
  RUN_TEST(test_cli_enter_interactive);
  RUN_TEST(test_cli_enter_leave_non_interactive);
  RUN_TEST(test_cli_configurator_handshake);
  RUN_TEST(test_cli_process_comment);
  RUN_TEST(test_cli_overflow);
  RUN_TEST(test_cli_process_help);
  RUN_TEST(test_cli_process_help_non_interactive);
  RUN_TEST(test_cli_get_mixer_type);
  RUN_TEST(test_cli_set_mixer_type);
  RUN_TEST(test_cli_bf_get_mag_calibration);
  RUN_TEST(test_cli_bf_sensor_hardware);
  return UNITY_END();
}