/*!
\file socket.cpp
\brief Gestione della comunicazione client/server mediante socket.

Sul PCN gira un programma (imgserver) che aspetta delle richieste TCP/IP sulla porta 5400 
mediante la chiamata alla funzione listen(); 
esso accetta una sola connessione alla volta mediante la chiamata alla funzione accept(). 
Quando la connessione viene stabilita, il client (win_client) pu&ograve; 
inviare comandi per (ad esempio) acquisire le immagini 
mediante una chiamata a SendString() e una chiamata a Send() che servono rispettivamente 
per comunicare il tipo di comando da eseguire e gli eventuali parametri associati a quel specifico comando, 
Oltre all'acquisizione &egrave; possibile settare la data e l'ora, forzare il salvataggio del file di log, 
ricevere informazioni, ecc. 
Se il protocollo TCP sulla porta 5400 &egrave; usato per controllare il dispositivo, 
il protocollo UDP sulla porta 5402 viene usato dal PCN per inviare le immagini 
e i due contatori al client mediante una serie di chiamate della funzione sendto(). 

%Per ulteriori dettagli sulla gestione delle socket vedere la funzione main() e main_loop().

<B>Apertura della connessione lato client</B>

Si riporta codice dell'applicativo win_client:

\code 
#include <winsock2.h> 
#define PORT 5400 
SOCKET sock; 
struct sockaddr_in serv_addr; 

// creazione canale di comunicazione
sock = socket(AF_INET,SOCK_STREAM,0); 

memset ((char *)&serv_addr,0,sizeof(serv_addr)); 
serv_addr.sin_family = AF_INET; 
serv_addr.sin_port   = htons(PORT);                    // service port 
serv_addr.sin_addr.s_addr = inet_addr("192.168.0.2");  // PCN-1001 IP address 

// il client si connette alla socket
connect(sock,(struct sockaddr *)&serv_addr, sizeof(sockaddr));
\endcode


<B>Creazione della socket lato server e attesa della richiesta di un client</B>

\code
// creazione canale di comunicazione
if ((listener = socket(AF_INET, SOCK_STREAM, 0)) == -1) exit(1);

// il server aggiunge delle opzioni alla socket
if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes,sizeof(int)) == -1) exit(1);

myaddr.sin_family = AF_INET;
myaddr.sin_addr.s_addr = INADDR_ANY;
myaddr.sin_port = htons(PORT);
memset(&(myaddr.sin_zero), '\0', 8);

// il server lega un indirizzo alla socket creata prima
if (bind(listener, (struct sockaddr *)&myaddr, sizeof(myaddr)) == -1) exit(1);

// il server resta in ascolto di una richiesta fatta da un client
// ad esempio mediante l'interfaccia utente win_client
if (listen(listener, MAX_CONN) == -1) exit(1);
\endcode

\author Alessio Negri, Andrea Colombari (eVS - embedded Vision Systems s.r.l. - http://www.evsys.net)
*/


/*!
*  \brief Invio del buffer tramite socket TCP.

Internamente viene eseguita una serie di send(la funzione per l'invio dei pacchetti scambiati mediante socket TCP) 
controllando il numero effettivo di bytes inviati 
per essere sicuri di aver inviato tutto il messaggio. Se la funzione send() ritorna 0 allora 
viene restituito il controllo alla funzione chiamante.

*  \param fd file descriptor.
*  \param buf data buffer.
*  \param len number of byte to send.
*  \return number of sended byte.
*/
int Send(int fd,void *buf,int len)
{
    int sum = 0;
    int bytes = 0;
    int bytesleft;
    char *buffer;

    buffer = (char *)buf;
    bytesleft = len;
    while(sum < len)
    {
        bytes = send(fd,buffer+sum,bytesleft, 0);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
        else return bytes;
    }
    return sum;
}


/*! 
*  \brief Ricezione del buffer tramite socket TCP.

Internamente viene eseguita una serie di recv(la funzione per la ricezione dei pacchetti scambiati mediante socket TCP) 
controllando il numero effettivo di bytes ricevuti 
per essere sicuri di aver ricevuto tutto il messaggio. Se la funzione recv() ritorna 0 allora 
viene restituito il controllo alla funzione chiamante.

*  \param fd file descriptor.
*  \param buf data buffer.
*  \param len number of byte to receive.
*  \return number of received byte.
*/
int Recv(int fd,void *buf,int len)
{
    int sum = 0;
    int bytes = 0;
    int bytesleft;
    char *buffer;

    buffer = (char *)buf;
    bytesleft = len;
    while(sum < len)
    {
        bytes = recv(fd,buffer+sum,bytesleft, 0);
        if(bytes > 0) {sum += bytes; bytesleft -= bytes;}
        else return bytes;
    }
    return sum;
}


/*! 
*  \brief Ricezione di una stringa tramite socket TCP.

Come nel caso della Recv() anche questa funzione si basa sulla recv() per ricevere un pacchetto mediante socket TCP.

*  \param fd file descriptor.
*  \param buf string buffer.
*  \param maxlen is a maximum number of character to receive.
*  \return number of received character.
*/
int RecvString(int fd,char *buf,int maxlen)
{
    int sum = 0;
    int bytes = 0;
    int counter;
    char a;

    strcpy(buf,"\0");
    counter = 0;
    a = '1';
    while(a && sum < maxlen)
    {
        bytes = recv(fd,&a,1, 0);
        if(bytes > 0)
        {
            buf[counter] = a;
            counter++;
            sum += bytes;
        }
        else return bytes;
    }
    return sum;
}


/*! 
*  \brief Invio di una stringa tramite socket TCP.

Come nel caso della Send() anche questa funzione si basa sulla send() per inviare un pacchetto mediante socket TCP.

*  \param fd file descriptor.
*  \param buf string buffer.
*  \return number of sended character.
*/
int SendString(int fd,char *buf)
{
    int bytes;
    int i;

    i = 0;
    while(buf[i++] && i < MAX_STR_LENGTH);

    bytes = send(fd,buf,i, 0);

    return bytes;
}



/* 
*  \brief Connect create the socket and to connect at the created socket.
*  \param sockfd socket descriptor.
*  \param addr address.
*  \return If the connection is valid then return zero (-1 otherwise).
*/
int Connect(int *sockfd,char *addr)
{
    struct hostent *he;
    struct sockaddr_in their_addr; 

    if ((he=gethostbyname(addr)) == NULL)
    {  
        perror("gethostbyname");
        return -1;
    }

    if ((*sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        return -1;
    }

    their_addr.sin_family = AF_INET;    
    their_addr.sin_port = htons(PORT);  
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    memset(&(their_addr.sin_zero), '\0', 8); 

    if (connect(*sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1)
    {
        perror("connect");
        return -1;
    }
    return 0;
}


/* 
*  \brief Close the created socket.
*  \param sockfd socket descriptor.
*/
void Disconnect(int *sockfd)
{
    close(*sockfd);
}

