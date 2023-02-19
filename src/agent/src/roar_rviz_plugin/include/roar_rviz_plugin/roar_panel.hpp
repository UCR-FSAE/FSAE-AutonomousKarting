#ifndef ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_
#define ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>
#include "rviz_common/panel.hpp"
#include <nav2_lifecycle_manager/lifecycle_manager.hpp>

class QPushButton;

namespace roar_rviz_plugin
{
    class ROARPanel : public rviz_common::Panel
    {
    public:
        explicit ROARPanel(QWidget *parent = 0);
        virtual ~ROARPanel();

        void onInitialize() override;

        /// Load and save configuration data
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
    
    private:
        void setupUI();
        void handleConfigureClicked();
        void handleActivateClicked();
        void handleDeactivateClicked();
        void handleCleanup();

        QPushButton * configure_button_{nullptr};
        QPushButton * activate_button_{nullptr};
        QPushButton * deactivate_button_{nullptr};
        QPushButton * cleanup_button_{nullptr};
        std::vector<std::string> * lifecycle_nodes;
    };
}
#endif //  ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_
