#ifndef DATABASEMANAGER_H
#define DATABASEMANAGER_H

#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <memory>
#include <vector>
#include "models/Task.h"

class DatabaseManager : public QObject {
    Q_OBJECT

public:
    static DatabaseManager& instance();
    
    // 删除拷贝构造函数和赋值操作符
    DatabaseManager(const DatabaseManager&) = delete;
    DatabaseManager& operator=(const DatabaseManager&) = delete;

    // 数据库操作
    bool initializeDatabase();
    bool addTask(const Task& task);
    bool updateTask(const Task& task);
    bool deleteTask(int id);
    std::vector<Task> getAllTasks() const;
    std::unique_ptr<Task> getTaskById(int id) const;
    
    // 数据库状态
    bool isDatabaseOpen() const;
    QString getLastError() const;

private:
    explicit DatabaseManager(QObject* parent = nullptr);
    ~DatabaseManager();
    
    bool createTables();
    Task createTaskFromQuery(const QSqlQuery& query) const;
    
    QSqlDatabase m_database;
    mutable QString m_lastError;
};

#endif // DATABASEMANAGER_H