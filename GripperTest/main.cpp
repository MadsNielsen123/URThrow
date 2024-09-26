#include <QCoreApplication>
#include <QTcpSocket>
#include <QtDebug>

int main()
{
    QTcpSocket socket;
    socket.connectToHost("192.168.1.20", 1000);
    if(socket.waitForConnected(5000))
    {
        QString command = "home()\n";
        socket.write(command.toUtf8()); //Send

        if(!socket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
        {
            qDebug() << "Failed to send command, exiting: " << socket.errorString();
            return 0;
        }

        if(!socket.waitForReadyRead(3000))
            qDebug() << "Command not received: " << socket.errorString();

        QByteArray response = socket.readLine(); //Command ACK

        if(!socket.waitForReadyRead(5000))
            qDebug() << "Command not done " << socket.errorString();
        response = socket.readLine(); //Command Finished

        command = "grip()\n";
        socket.write(command.toUtf8()); //SACK GRIP


        if(!socket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
        {
            qDebug() << "Failed to send command, exiting: " << socket.errorString();
            return 0;
        }

        if(!socket.waitForReadyRead(3000))
            qDebug() << "Command not received: " << socket.errorString();

        response = socket.readLine(); //Command ACK

        if(!socket.waitForReadyRead(5000))
            qDebug() << "Command not done " << socket.errorString();
        response = socket.readLine(); //Command Finished

        command = "release(40,200)\n";
        socket.write(command.toUtf8()); //SACK GRIP


        if(!socket.waitForBytesWritten(3000)) //If hasn't been send in 3s -> failed
        {
            qDebug() << "Failed to send command, exiting: " << socket.errorString();
            return 0;
        }

        if(!socket.waitForReadyRead(3000))
            qDebug() << "Command not received: " << socket.errorString();

        response = socket.readLine(); //Command ACK

        if(!socket.waitForReadyRead(5000))
            qDebug() << "Command not done " << socket.errorString();
        response = socket.readLine(); //Command Finished

        qDebug() << "Done";

    }
    else
        qDebug() << "Failed to connect: " << socket.errorString();


    return 0;
}
