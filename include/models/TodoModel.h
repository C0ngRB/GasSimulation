#ifndef TODOMODEL_H
#define TODOMODEL_H

#include <QAbstractListModel>
#include <vector>
#include <memory>
#include "Task.h"

class DatabaseManager;

class TodoModel : public QAbstractListModel {
    Q_OBJECT

public:
    enum TodoRoles {
        IdRole = Qt::UserRole + 1,
        TitleRole,
        DescriptionRole,
        CompletedRole,
        CreatedAtRole,
        UpdatedAtRole
    };

    explicit TodoModel(QObject* parent = nullptr);
    ~TodoModel() override = default;

    // 重写QAbstractListModel的方法
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QHash<int, QByteArray> roleNames() const override;

    // 自定义方法
    Q_INVOKABLE bool addTask(const QString& title, const QString& description = "");
    Q_INVOKABLE bool removeTask(int index);
    Q_INVOKABLE bool toggleTaskCompleted(int index);
    Q_INVOKABLE void refresh();
    
    // 获取任务
    Task getTask(int index) const;
    bool updateTask(int index, const Task& task);

signals:
    void errorOccurred(const QString& error);
    void taskAdded(int index);
    void taskRemoved(int index);
    void taskUpdated(int index);

private:
    void loadTasksFromDatabase();
    bool saveTaskToDatabase(const Task& task);
    bool deleteTaskFromDatabase(int id);
    bool updateTaskInDatabase(const Task& task);
    
    std::vector<Task> m_tasks;
    DatabaseManager& m_dbManager;
};

#endif // TODOMODEL_H