#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "cpp_rosbridge_core/resource_proxy.hpp"

namespace fs = std::filesystem;

class ResourceProxyTest : public ::testing::Test {
protected:
    void SetUp() override {
        cache_root_ = fs::temp_directory_path() / "tachybridge_resource_proxy_test";
        fs::remove_all(cache_root_);
        fs::create_directories(cache_root_ / "192.168.0.10" / "remote_pkg" / "meshes");
        std::ofstream(cache_root_ / "192.168.0.10" / "remote_pkg" / "meshes" / "link.stl")
            << "solid mesh";
    }

    void TearDown() override {
        fs::remove_all(cache_root_);
    }

    fs::path cache_root_;
};

TEST_F(ResourceProxyTest, RewritesPackageUrisWhenCachedResourceExists) {
    cpp_rosbridge_core::ResourceProxy::Config config;
    config.enabled = true;
    config.prefer_local_packages = false;
    config.robot_ip = "192.168.0.10";
    config.local_base_uri = "/resources";
    config.cache_root = cache_root_.string();

    cpp_rosbridge_core::ResourceProxy proxy(config);
    const std::string xml =
        R"(<robot><link><visual><geometry><mesh filename="package://remote_pkg/meshes/link.stl"/></geometry></visual></link></robot>)";

    const auto rewritten = proxy.rewrite_robot_description(xml);
    ASSERT_TRUE(rewritten.has_value());
    EXPECT_NE(rewritten->find("/resources/192.168.0.10/remote_pkg/meshes/link.stl"), std::string::npos);
}

TEST_F(ResourceProxyTest, SkipsRewriteWhenDisabled) {
    cpp_rosbridge_core::ResourceProxy::Config config;
    config.enabled = false;
    config.robot_ip = "192.168.0.10";
    config.cache_root = cache_root_.string();

    cpp_rosbridge_core::ResourceProxy proxy(config);
    const std::string xml =
        R"(<mesh filename="package://remote_pkg/meshes/link.stl"/>)";

    const auto rewritten = proxy.rewrite_robot_description(xml);
    EXPECT_FALSE(rewritten.has_value());
}
