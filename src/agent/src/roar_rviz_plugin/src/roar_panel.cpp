#include "roar_rviz_plugin/roar_panel.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <nav2_util/lifecycle_service_client.hpp>

namespace roar_rviz_plugin
{

    ROARPanel::ROARPanel(QWidget *parent)
        : Panel(parent)
    {
        this->lifecycle_nodes = new std::vector<std::string>;
        this->lifecycle_nodes->push_back("/costmap_node_manager");
        this->lifecycle_nodes->push_back("/global_planner_manager");
        this->lifecycle_nodes->push_back("/simple_local_planner");

        this->setupUI();
    }

    ROARPanel::~ROARPanel()
    {
    }

    void ROARPanel::setupUI()
    {
        configure_button_ = new QPushButton;
        activate_button_ = new QPushButton;
        deactivate_button_ = new QPushButton;
        cleanup_button_ = new QPushButton;

        configure_button_->setText(QString("Configure"));
        activate_button_->setText(QString("Activate"));
        deactivate_button_->setText(QString("Deactivate"));
        cleanup_button_->setText(QString("Cleanup"));

        connect(configure_button_, &QPushButton::released, this, &ROARPanel::handleConfigureClicked);
        connect(activate_button_, &QPushButton::released, this, &ROARPanel::handleActivateClicked);
        connect(deactivate_button_, &QPushButton::released, this, &ROARPanel::handleDeactivateClicked);
        connect(cleanup_button_, &QPushButton::released, this, &ROARPanel::handleCleanup);

        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->addWidget(configure_button_);
        main_layout->addWidget(activate_button_);
        main_layout->addWidget(deactivate_button_);
        main_layout->addWidget(cleanup_button_);
        main_layout->setContentsMargins(10, 10, 10, 10);
        setLayout(main_layout);
    }
    void ROARPanel::onInitialize()
    {
    }

    void
    ROARPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void
    ROARPanel::load(const rviz_common::Config &config)
    {
        Panel::load(config);
    }

    void
    ROARPanel::handleConfigureClicked() 
    {
        for (const auto &current_string : *this->lifecycle_nodes)
        {
            std::cout << "--- Configuring " << current_string.c_str() << " ---" << std::endl;
            auto curr_command = "ros2 lifecycle set " + current_string + " configure";
            system(curr_command.c_str());
        }

        std::cout << "Attempted to configure " << this->lifecycle_nodes->size() << " nodes" << std::endl;
    }

    void
    ROARPanel::handleActivateClicked()
    {
        for (const auto &current_string : *this->lifecycle_nodes)
        {
            std::cout << "--- Activating " << current_string.c_str() << " ---" << std::endl;
            auto curr_command = "ros2 lifecycle set " + current_string + " activate";
            system(curr_command.c_str());
        }

        std::cout << "Attempted to activate " << this->lifecycle_nodes->size() << " nodes" << std::endl;
    }

    void
    ROARPanel::handleDeactivateClicked()
    {
        for (const auto &current_string : *this->lifecycle_nodes)
        {
            std::cout << "--- Deactivating " << current_string.c_str() << " ---" << std::endl;
            auto curr_command = "ros2 lifecycle set " + current_string + " deactivate";
            system(curr_command.c_str());
        }

        std::cout << "Attempted to deactivate " << this->lifecycle_nodes->size() << " nodes" << std::endl;
    }

    void
    ROARPanel::handleCleanup()
    {
        for (const auto &current_string : *this->lifecycle_nodes)
        {
            std::cout << "--- Cleaning up " << current_string.c_str() << " ---" << std::endl;
            auto curr_command = "ros2 lifecycle set " + current_string + " cleanup";
            system(curr_command.c_str());
        }

        std::cout << "Attempted to cleanup " << this->lifecycle_nodes->size() << " nodes" << std::endl;
    }
} // namespace roar_rviz_plugin
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(roar_rviz_plugin::ROARPanel, rviz_common::Panel)