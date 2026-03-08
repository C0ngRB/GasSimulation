#include "database/DatabaseManager.h"
#include <QStandardPaths>
#include <QDir>
#include <QDebug>

DatabaseManager& DatabaseManager::instance() {
    static DatabaseManager instance;
    return instance;
}

DatabaseManager::DatabaseManager(QObject* parent) : QObject(parent) {
    initializeDatabase();
}

DatabaseManager::~DatabaseManager() {
    if (m_database.isOpen()) {
        m_database.close();
    }
}

bool DatabaseManager::initializeDatabase() {
    // 获取应用程序数据目录
    QString dataPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir dir(dataPath);
    if (!dir.exists()) {
        dir.mkpath(dataPath);
    }
    
    QString dbPath = dir.filePath("todos.db");
    
    m_database = QSqlDatabase::addDatabase("QSQLITE");
    m_database.setDatabaseName(dbPath);
    
    if (!m_database.open()) {
        m_lastError = m_database.lastError().text();
        qDebug() << "Failed to open database:" << m_lastError;
        return false;
    }
    
    return createTables();
}

bool DatabaseManager::createTables() {
    QSqlQuery query;
    
    QString createTableQuery = R"(
        CREATE TABLE IF NOT EXISTS tasks (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            title TEXT NOT NULL,
            description TEXT DEFAULT '',
            completed BOOLEAN DEFAULT 0,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
            updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    )";
    
    if (!query.exec(createTableQuery)) {
        m_lastError = query.lastError().text();
        qDebug() << "Failed to create table:" << m_lastError;
        return false;
    }
    
    return true;
}

bool DatabaseManager::addTask(const Task& task) {
    QSqlQuery query;
    query.prepare(R"(
        INSERT INTO tasks (title, description, completed, created_at, updated_at)
        VALUES (?, ?, ?, ?, ?)
    )");
    
    query.addBindValue(task.title);
    query.addBindValue(task.description);
    query.addBindValue(task.completed);
    query.addBindValue(task.createdAt);
    query.addBindValue(task.updatedAt);
    
    if (!query.exec()) {
        m_lastError = query.lastError().text();
        qDebug() << "Failed to add task:" << m_lastError;
        return false;
    }
    
    return true;
}

bool DatabaseManager::updateTask(const Task& task) {
    QSqlQuery query;
    query.prepare(R"(
        UPDATE tasks SET
            title = ?,
            description = ?,
            completed = ?,
            updated_at = ?
        WHERE id = ?
    )");
    
    query.addBindValue(task.title);
    query.addBindValue(task.description);
    query.addBindValue(task.completed);
    query.addBindValue(QDateTime::currentDateTime());
    query.addBindValue(task.id);
    
    if (!query.exec()) {
        m_lastError = query.lastError().text();
        qDebug() << "Failed to update task:" << m_lastError;
        return false;
    }
    
    return query.numRowsAffected() > 0;
}

bool DatabaseManager::deleteTask(int id) {
    QSqlQuery query;
    query.prepare("DELETE FROM tasks WHERE id = ?");
    query.addBindValue(id);
    
    if (!query.exec()) {
        m_lastError = query.lastError().text();
        qDebug() << "Failed to delete task:" << m_lastError;
        return false;
    }
    
    return query.numRowsAffected() > 0;
}

std::vector<Task> DatabaseManager::getAllTasks() const {
    std::vector<Task> tasks;
    QSqlQuery query("SELECT id, title, description, completed, created_at, updated_at FROM tasks ORDER BY created_at DESC");
    
    while (query.next()) {
        tasks.push_back(createTaskFromQuery(query));
    }
    
    if (query.lastError().isValid()) {
        m_lastError = query.lastError().text();
        qDebug() << "Failed to get all tasks:" << m_lastError;
    }
    
    return tasks;
}

std::unique_ptr<Task> DatabaseManager::getTaskById(int id) const {
    QSqlQuery query;
    query.prepare("SELECT id, title, description, completed, created_at, updated_at FROM tasks WHERE id = ?");
    query.addBindValue(id);
    
    if (!query.exec() || !query.next()) {
        m_lastError = query.lastError().text();
        return nullptr;
    }
    
    return std::make_unique<Task>(createTaskFromQuery(query));
}

bool DatabaseManager::isDatabaseOpen() const {
    return m_database.isOpen();
}

QString DatabaseManager::getLastError() const {
    return m_lastError;
}

Task DatabaseManager::createTaskFromQuery(const QSqlQuery& query) const {
    return Task(
        query.value("id").toInt(),
        query.value("title").toString(),
        query.value("description").toString(),
        query.value("completed").toBool(),
        query.value("created_at").toDateTime(),
        query.value("updated_at").toDateTime()
    );
}