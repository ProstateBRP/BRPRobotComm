#ifndef __ProstateRobotCommunicationBase_HPP_
#define __ProstateRobotCommunicationBase_HPP_

#include "igtlSocket.h"
#include "igtlMath.h"
#include "igtlMessageBase.h"

class ProstateRobotCommunicationBase
{
public:
  enum
  {
    SUCCESS = 1
  };

public:
  ProstateRobotCommunicationBase();
  virtual ~ProstateRobotCommunicationBase(); // changed the destructor to virtual

  virtual const char *Name() = 0;

  void SetSocket(igtl::Socket *socket);
  int ReceiveMessageHeader(igtl::MessageHeader *headerMsg, bool timeout);
  int SkipMesage(igtl::MessageHeader *headerMsg);
  void GetRandomTestMatrix(igtl::Matrix4x4 &matrix);
  int SendStringMessage(const char *name, const char *string);
  int SendStatusMessage(const char *name, int Code, int SubCode,
                        const char *errorName = NULL, const char *statusString = NULL);
  int SendTransformMessage(const char *name, igtl::Matrix4x4 &matrix);

  int CheckAndReceiveStringMessage(igtl::MessageHeader *headerMsg,
                                   const char *name, const char *string, int suffix = 0);
  int CheckAndReceiveStatusMessage(igtl::MessageHeader *headerMsg,
                                   const char *name, int code, int suffix = 0,
                                   const char *errorName = NULL);
  int CheckAndReceiveTransformMessage(igtl::MessageHeader *headerMsg,
                                      const char *name, igtl::Matrix4x4 &matrix,
                                      double err = 1.0e-10, int suffix = 0);

  int ReceiveTransform(igtl::MessageHeader *header, igtl::Matrix4x4 &matrix);
  int ReceiveString(igtl::MessageHeader *header, std::string &string);
  int ReceiveStatus(igtl::MessageHeader *header, int &code, int &subcode,
                    std::string &name, std::string &status);

  void PrintMatrix(std::string prefix,const igtl::Matrix4x4 &matrix);
  int ValidateMatrix(const igtl::Matrix4x4 &matrix);

  // Compare two matrices. If there is any corresponding elements with error larger than 'tol', return 0.
  // Otherwise, it returns 1.
  int CompareMatrices(const igtl::Matrix4x4 &matrix1, const igtl::Matrix4x4 &matrix2, double tol);
  bool connect{false}; // Shows the status of socket connection

protected:
  igtl::Socket::Pointer Socket;
};

#endif //__ProstateRobotCommunicationBase_HPP_
