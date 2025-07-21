#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "foundation_common/base_component.hpp"

class TestComponent : public foundation_common::BaseComponent
{
public:
  TestComponent() : BaseComponent("test_component") {}
};

class TestBaseComponent : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestBaseComponent, ComponentCreation)
{
  auto component = std::make_shared<TestComponent>();
  ASSERT_NE(component, nullptr);
  EXPECT_EQ(component->get_name(), std::string("test_component"));
}

TEST_F(TestBaseComponent, LifecycleTransitions)
{
  auto component = std::make_shared<TestComponent>();
  
  // Test initial state
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::UNINITIALIZED);
  
  // Test configure transition
  auto result = component->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(result, rclcpp_lifecycle::CallbackReturn::SUCCESS);
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::CONFIGURED);
  
  // Test activate transition
  result = component->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(result, rclcpp_lifecycle::CallbackReturn::SUCCESS);
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::ACTIVE);
  
  // Test deactivate transition
  result = component->on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(result, rclcpp_lifecycle::CallbackReturn::SUCCESS);
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::INACTIVE);
}

TEST_F(TestBaseComponent, StatusSetting)
{
  auto component = std::make_shared<TestComponent>();
  
  component->setComponentStatus(foundation_common::BaseComponent::ComponentStatus::ERROR);
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::ERROR);
  
  component->setComponentStatus(foundation_common::BaseComponent::ComponentStatus::ACTIVE);
  EXPECT_EQ(component->getComponentStatus(), 
            foundation_common::BaseComponent::ComponentStatus::ACTIVE);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
