/********************************************************************************
** Form generated from reading UI file 'importdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_IMPORTDIALOG_H
#define UI_IMPORTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ImportDialog
{
public:
    QGridLayout *gridLayout;
    QDialogButtonBox *buttonBox;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *csvEdit;
    QPushButton *csvButton;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *elfEdit;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLineEdit *kallsymsEdit;

    void setupUi(QDialog *ImportDialog)
    {
        if (ImportDialog->objectName().isEmpty())
            ImportDialog->setObjectName(QStringLiteral("ImportDialog"));
        ImportDialog->resize(400, 300);
        gridLayout = new QGridLayout(ImportDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        buttonBox = new QDialogButtonBox(ImportDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 1, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_3 = new QLabel(ImportDialog);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_4->addWidget(label_3);

        csvEdit = new QLineEdit(ImportDialog);
        csvEdit->setObjectName(QStringLiteral("csvEdit"));

        horizontalLayout_4->addWidget(csvEdit);

        csvButton = new QPushButton(ImportDialog);
        csvButton->setObjectName(QStringLiteral("csvButton"));

        horizontalLayout_4->addWidget(csvButton);


        horizontalLayout_2->addLayout(horizontalLayout_4);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(ImportDialog);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        elfEdit = new QLineEdit(ImportDialog);
        elfEdit->setObjectName(QStringLiteral("elfEdit"));

        horizontalLayout->addWidget(elfEdit);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_2 = new QLabel(ImportDialog);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_3->addWidget(label_2);

        kallsymsEdit = new QLineEdit(ImportDialog);
        kallsymsEdit->setObjectName(QStringLiteral("kallsymsEdit"));

        horizontalLayout_3->addWidget(kallsymsEdit);


        verticalLayout->addLayout(horizontalLayout_3);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);


        retranslateUi(ImportDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ImportDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ImportDialog, SLOT(reject()));
        QObject::connect(csvButton, SIGNAL(clicked()), ImportDialog, SLOT(updateCsv()));

        QMetaObject::connectSlotsByName(ImportDialog);
    } // setupUi

    void retranslateUi(QDialog *ImportDialog)
    {
        ImportDialog->setWindowTitle(QApplication::translate("ImportDialog", "Dialog", Q_NULLPTR));
        label_3->setText(QApplication::translate("ImportDialog", "CSV file", Q_NULLPTR));
        csvButton->setText(QApplication::translate("ImportDialog", "...", Q_NULLPTR));
        label->setText(QApplication::translate("ImportDialog", "Elf files (comma separated)", Q_NULLPTR));
        label_2->setText(QApplication::translate("ImportDialog", "kallsyms file", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ImportDialog: public Ui_ImportDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_IMPORTDIALOG_H
