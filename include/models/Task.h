#ifndef TASK_H
#define TASK_H

#include <QString>
#include <QDateTime>
#include <QMetaType>

struct Task {
    int id;
    QString title;
    QString description;
    bool completed;
    QDateTime createdAt;
    QDateTime updatedAt;

    Task() : id(-1), completed(false) {}
    
    Task(int id, const QString& title, const QString& description = "", 
         bool completed = false, const QDateTime& createdAt = QDateTime::currentDateTime(),
         const QDateTime& updatedAt = QDateTime::currentDateTime())
        : id(id), title(title), description(description), completed(completed),
          createdAt(createdAt), updatedAt(updatedAt) {}

    bool isValid() const { return id >= 0; }
};

Q_DECLARE_METATYPE(Task)

#endif // TASK_H