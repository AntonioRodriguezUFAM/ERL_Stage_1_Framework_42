/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionUpgrade_PMU_Firmware;
    QAction *actionAbout;
    QAction *actionExit;
    QAction *actionProfile;
    QAction *actionImport_CSV;
    QAction *actionExport_CSV;
    QAction *actionLynsyn_HW_information;
    QAction *actionLive;
    QAction *actionJTAG_diagnostic;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(800, 600);
        actionUpgrade_PMU_Firmware = new QAction(MainWindow);
        actionUpgrade_PMU_Firmware->setObjectName(QStringLiteral("actionUpgrade_PMU_Firmware"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QStringLiteral("actionProfile"));
        actionImport_CSV = new QAction(MainWindow);
        actionImport_CSV->setObjectName(QStringLiteral("actionImport_CSV"));
        actionExport_CSV = new QAction(MainWindow);
        actionExport_CSV->setObjectName(QStringLiteral("actionExport_CSV"));
        actionLynsyn_HW_information = new QAction(MainWindow);
        actionLynsyn_HW_information->setObjectName(QStringLiteral("actionLynsyn_HW_information"));
        actionLive = new QAction(MainWindow);
        actionLive->setObjectName(QStringLiteral("actionLive"));
        actionJTAG_diagnostic = new QAction(MainWindow);
        actionJTAG_diagnostic->setObjectName(QStringLiteral("actionJTAG_diagnostic"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));

        gridLayout->addWidget(tabWidget, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 30));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        toolBar_2 = new QToolBar(MainWindow);
        toolBar_2->setObjectName(QStringLiteral("toolBar_2"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar_2);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionImport_CSV);
        menuFile->addAction(actionExport_CSV);
        menuFile->addSeparator();
        menuFile->addAction(actionUpgrade_PMU_Firmware);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuHelp->addAction(actionLynsyn_HW_information);
        menuHelp->addAction(actionJTAG_diagnostic);
        menuHelp->addSeparator();
        menuHelp->addAction(actionAbout);
        toolBar->addAction(actionProfile);
        toolBar->addAction(actionLive);

        retranslateUi(MainWindow);
        QObject::connect(actionAbout, SIGNAL(triggered()), MainWindow, SLOT(about()));
        QObject::connect(actionExit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(actionProfile, SIGNAL(triggered()), MainWindow, SLOT(profileEvent()));
        QObject::connect(actionImport_CSV, SIGNAL(triggered()), MainWindow, SLOT(importCsv()));
        QObject::connect(actionExport_CSV, SIGNAL(triggered()), MainWindow, SLOT(exportCsv()));
        QObject::connect(actionUpgrade_PMU_Firmware, SIGNAL(triggered()), MainWindow, SLOT(upgrade()));
        QObject::connect(actionLynsyn_HW_information, SIGNAL(triggered()), MainWindow, SLOT(showHwInfo()));
        QObject::connect(actionLive, SIGNAL(triggered()), MainWindow, SLOT(showLive()));
        QObject::connect(actionJTAG_diagnostic, SIGNAL(triggered()), MainWindow, SLOT(jtagDiagnostic()));

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        actionUpgrade_PMU_Firmware->setText(QApplication::translate("MainWindow", "&Upgrade PMU Firmware", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("MainWindow", "&About", Q_NULLPTR));
        actionExit->setText(QApplication::translate("MainWindow", "&Exit", Q_NULLPTR));
        actionProfile->setText(QApplication::translate("MainWindow", "Profile", Q_NULLPTR));
        actionImport_CSV->setText(QApplication::translate("MainWindow", "&Import CSV", Q_NULLPTR));
        actionExport_CSV->setText(QApplication::translate("MainWindow", "Export &CSV", Q_NULLPTR));
        actionLynsyn_HW_information->setText(QApplication::translate("MainWindow", "&Lynsyn HW information", Q_NULLPTR));
        actionLive->setText(QApplication::translate("MainWindow", "Live", Q_NULLPTR));
        actionJTAG_diagnostic->setText(QApplication::translate("MainWindow", "JTAG diagnostic", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "Fi&le", Q_NULLPTR));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", Q_NULLPTR));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", Q_NULLPTR));
        toolBar_2->setWindowTitle(QApplication::translate("MainWindow", "toolBar_2", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
