/**
 * @file test_proc_spec.cpp
 * @brief 受管进程规格解析与 tmux 命令行构造单测（零 ROS）
 */

#include <gtest/gtest.h>

#include "ros_mqtt_bridge/proc_spec.h"

using ros_mqtt_bridge::buildLaunchScript;
using ros_mqtt_bridge::paneOutputAlive;
using ros_mqtt_bridge::parseProcSpec;
using ros_mqtt_bridge::ProcSpec;
using ros_mqtt_bridge::tmuxHasSessionArgs;
using ros_mqtt_bridge::tmuxKillSessionArgs;
using ros_mqtt_bridge::tmuxListPanesArgs;
using ros_mqtt_bridge::tmuxNewSessionArgs;

// ── parseProcSpec ────────────────────────────────────────────

TEST(ProcSpec, ParseFullTriple)
{
    const ProcSpec s = parseProcSpec(
        "ros2 launch uav_guide uav_guide_launch.py|uav_guide_launch.py|aoa_nav", "fb");
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.command, "ros2 launch uav_guide uav_guide_launch.py");
    EXPECT_EQ(s.pattern, "uav_guide_launch.py");
    EXPECT_EQ(s.session, "aoa_nav");
}

TEST(ProcSpec, ParseWithoutSessionUsesFallback)
{
    const ProcSpec s = parseProcSpec("ros2 run ros_mqtt_bridge mqtt_control_bridge|mqtt_control_bridge", "pfx_1");
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.session, "pfx_1");
    EXPECT_EQ(s.pattern, "mqtt_control_bridge");
}

TEST(ProcSpec, ParseWithoutPatternFallsBackToSession)
{
    const ProcSpec s = parseProcSpec("ros2 launch foo bar.launch.py", "pfx_2");
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.pattern, "pfx_2");
    EXPECT_EQ(s.session, "pfx_2");
}

TEST(ProcSpec, ParseTrimsWhitespace)
{
    const ProcSpec s = parseProcSpec("  cmd  |  pat  |  sess  ", "fb");
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.command, "cmd");
    EXPECT_EQ(s.pattern, "pat");
    EXPECT_EQ(s.session, "sess");
}

TEST(ProcSpec, ParseRejectsEmptyCommand)
{
    EXPECT_FALSE(parseProcSpec("", "fb").valid);
    EXPECT_FALSE(parseProcSpec("|pat|sess", "fb").valid);
    EXPECT_FALSE(parseProcSpec("   ", "fb").valid);
}

TEST(ProcSpec, ParseRejectsWhenNoSessionAvailable)
{
    // 无 session 段且无 fallback → 无效（避免生成匿名 tmux 会话）
    EXPECT_FALSE(parseProcSpec("cmd|pat|", "").valid);
}

// ── buildLaunchScript ───────────────────────────────────────

TEST(ProcSpec, LaunchScriptHasCdSourceExec)
{
    const std::string script = buildLaunchScript(
        "/home/ros-dev/beamDubins_ws",
        "source /opt/ros/humble/setup.bash && source install/setup.bash",
        "ros2 launch uav_guide uav_guide_launch.py");

    EXPECT_NE(script.find("cd /home/ros-dev/beamDubins_ws\n"), std::string::npos);
    EXPECT_NE(script.find("source /opt/ros/humble/setup.bash && source install/setup.bash\n"),
              std::string::npos);
    // 必须用 exec：让 tmux pane 的进程就是 ros2 本体
    EXPECT_NE(script.find("exec ros2 launch uav_guide uav_guide_launch.py\n"), std::string::npos);
    EXPECT_NE(script.find("set -e\n"), std::string::npos);
}

TEST(ProcSpec, LaunchScriptOmitsEmptyParts)
{
    const std::string script = buildLaunchScript("", "", "ros2 run pkg exe");
    EXPECT_EQ(script.find("cd "), std::string::npos);
    EXPECT_NE(script.find("exec ros2 run pkg exe\n"), std::string::npos);
}

TEST(ProcSpec, LaunchScriptTeesToLogWhenGiven)
{
    const std::string script = buildLaunchScript("/ws", "source a.bash", "ros2 run pkg exe",
                                                 "/tmp/logs/sess.log");
    EXPECT_NE(script.find("exec ros2 run pkg exe 2>&1 | tee -a /tmp/logs/sess.log\n"),
              std::string::npos);
    // 无 log 时不得出现 tee（保持纯 exec 语义）
    const std::string plain = buildLaunchScript("/ws", "source a.bash", "ros2 run pkg exe");
    EXPECT_EQ(plain.find("tee"), std::string::npos);
}

// ── tmux 参数构造 ───────────────────────────────────────────

TEST(ProcSpec, NewSessionArgs)
{
    const auto args = tmuxNewSessionArgs("aoa_nav", "/ws", "/tmp/x/aoa_nav.sh");
    ASSERT_EQ(args.size(), 7u);
    EXPECT_EQ(args[0], "new-session");
    EXPECT_EQ(args[1], "-d");
    EXPECT_EQ(args[2], "-s");
    EXPECT_EQ(args[3], "aoa_nav");
    EXPECT_EQ(args[4], "-c");
    EXPECT_EQ(args[5], "/ws");
    EXPECT_EQ(args[6], "bash '/tmp/x/aoa_nav.sh'");
}

TEST(ProcSpec, NewSessionArgsWithoutWorkspaceDir)
{
    const auto args = tmuxNewSessionArgs("s", "", "/tmp/s.sh");
    ASSERT_EQ(args.size(), 5u);
    EXPECT_EQ(args[3], "s");
    EXPECT_EQ(args[4], "bash '/tmp/s.sh'");
}

TEST(ProcSpec, KillHasListArgs)
{
    EXPECT_EQ(tmuxKillSessionArgs("aoa_bridge")[1], "-t");
    EXPECT_EQ(tmuxHasSessionArgs("aoa_bridge")[2], "aoa_bridge");
    const auto lp = tmuxListPanesArgs("aoa_bridge");
    ASSERT_EQ(lp.size(), 5u);
    EXPECT_EQ(lp[0], "list-panes");
    EXPECT_EQ(lp[1], "-t");
    EXPECT_EQ(lp[2], "aoa_bridge");
    EXPECT_EQ(lp[3], "-F");
    EXPECT_EQ(lp[4], "#{pane_dead}");
}

// ── paneOutputAlive（配合 remain-on-exit 判断真实存活）───────

TEST(ProcSpec, PaneAliveParsing)
{
    EXPECT_TRUE(paneOutputAlive("0"));        // 存活
    EXPECT_TRUE(paneOutputAlive("0\n"));
    EXPECT_TRUE(paneOutputAlive("1\n0\n"));   // 多 pane：任一存活即存活
    EXPECT_FALSE(paneOutputAlive("1"));       // 进程已退出（pane 被保留）
    EXPECT_FALSE(paneOutputAlive("1\n1\n"));
    EXPECT_FALSE(paneOutputAlive(""));        // 会话不存在
    EXPECT_FALSE(paneOutputAlive("\n \n"));
}
