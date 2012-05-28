///////////////////////////////////////////////////////////////////////////////////////
// This code ported to Ogre, (i.e. Torque Vector references changed to std::vector
// references, etc.) as part of the Ecstasy Motion Ogitor project.
//
//   Chris Calef, 2010
//   BrokeAss Games, LLC
//
///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// Torque Game Engine 
// Written by John Vanderbeck                                                  
//
// This code is written by John Vanderbeck and is offered freely to the Torque
// Game Engine wth no express warranties.  Use it for whatever you want, all
// I ask is that you don't rip it off and call it your own.  Credit where
// credit is due.  If you do use this, just drop me a line to let me know.  It
// makes me feel good :)
// Contact: jvanderbeck@novusdelta.com
//          http://www.novusdelta.com
//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
// This code implements support for SQLite into Torque and TorqueScript
//
// Essentially this creates a scriptable object that interfaces with SQLite.
//-----------------------------------------------------------------------------
//


#include "EcstasyMotion/SQLiteObject.h"

#include <Ogre.h>
#include "Ogitors.h"
#include "OgitorsRoot.h"
#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;

//#include "console/simBase.h"
//#include "console/consoleInternal.h"
#include <STDLIB.H>

//IMPLEMENT_CONOBJECT(SQLiteObject);

SQLiteObject::SQLiteObject()
{
   m_pDatabase = NULL;
   m_szErrorString = 0;
   m_iLastResultSet = 0;
   m_iNextResultSet = 1;
}

SQLiteObject::~SQLiteObject()
{
   // if we still have a database open, close it
   CloseDatabase();
   // Clear out any error string we may have left
   ClearErrorString();
   //// Clean up result sets
   ////
   //// Boy oh boy this is such a crazy hack!
   //// I can't seem to iterate through a vector and clean it up without screwing the vector.
   //// So (HACK HACK HACK) what i'm doing for now is making a temporary vector that
   //// contains a list of the result sets that the user hasn't cleaned up.
   //// Clean up all those result sets, then delete the temp vector.
   //Vector<int> vTemp;
   //Vector<int>::iterator iTemp;

   //VectorPtr<sqlite_resultset*>::iterator i;
   //for (i = m_vResultSets.begin(); i != m_vResultSets.end(); i++)
   //{
   //   vTemp.push_back((*i)->iResultSet);
   //}
   //index = 0;
   //for (iTemp = vTemp.begin(); iTemp != vTemp.end(); iTemp++)
   //{
   //   //Con::warnf("SQLiteObject Warning: Result set #%i was not cleared by script.  Clearing it now.", vTemp[index]);
   //   ClearResultSet(vTemp[index]);
   //   index++;
   //}

   //m_vResultSets.clear();

}

bool SQLiteObject::processArguments(int argc, const char **argv)
{
   if(argc == 0)
      return true;
   else 
      return true;

   return false;
}

//bool SQLiteObject::onAdd()
//{
   //if (!Parent::onAdd())
   //   return false;

   //const char *name = getName();
   //if(name && name[0] && getClassRep())
   //{
   //   Namespace *parent = getClassRep()->getNameSpace();
   //   Con::linkNamespaces(parent->mName, name);
   //   mNameSpace = Con::lookupNamespace(name);
   //
   //}

//   return true;
//}

// This is the function that gets called when an instance
// of your object is being removed from the system and being
// destroyed.  Use this to do your clean up and what not.
//void SQLiteObject::onRemove()
//{
//   CloseDatabase();
//   //Parent::onRemove();
//}

// To be honest i'm not 100% sure on when this is called yet.
// Basically its used to set the values of any persistant fields
// the object has.  Similiar to the way datablocks work.  I'm
// just not sure how and when this gets called.
//void SQLiteObject::initPersistFields()
//{
//   Parent::initPersistFields();
//}

//-----------------------------------------------------------------------
// These functions below are our custom functions that we will tie into
// script.

int Callback(void *pArg, int argc, char **argv, char **columnNames)
{
   // basically this callback is called for each row in the SQL query result.
   // for each row, argc indicates how many columns are returned.
   // columnNames[i] is the name of the column
   // argv[i] is the value of the column

   sqlite_resultrow* pRow;
   sqlite_resultset* pResultSet;
   char* name;
   char* value;
   int i;

   if (argc == 0)
      return 0;

   pResultSet = (sqlite_resultset*)pArg;
   if (!pResultSet)
      return -1;

   // create a new result row
   pRow = new sqlite_resultrow;
   pResultSet->iNumCols = argc;
   // loop through all the columns and stuff them into our row
   for (i = 0; i < argc; i++)
   {
      // DBEUG CODE
//      Con::printf("%s = %s\n", columnNames[i], argv[i] ? argv[i] : "NULL");
      name = new char[strlen(columnNames[i]) + 1];//+1?
      //strcpy_s(name, strlen(columnNames[i])+1, columnNames[i]);
		strcpy(name, columnNames[i]);
      pRow->vColumnNames.push_back(name);
      if (argv[i])
      {
         value = new char[strlen(argv[i]) + 1];
         strcpy(value, argv[i]);
         pRow->vColumnValues.push_back(value);
      }
      else
      {
         value = new char[10];
         strcpy(value,"NULL");
         pRow->vColumnValues.push_back(value);
      }
   }
   pResultSet->iNumRows++;
   pResultSet->vRows.push_back(pRow);
   
   return 0; //or else the sqlexec will be aborted.
}

bool SQLiteObject::OpenDatabase(const char* filename)
{
   // check to see if we already have an open database, and
   // if so, close it.
   CloseDatabase();

   // We persist the error string so that the script may make a
   // GetLastError() call at any time.  However when we get
   // ready to make a call which could result in a new error, 
   // we need to clear what we have to avoid a memory leak.
   ClearErrorString();

   m_pDatabase = sqlite_open(filename, 0, &m_szErrorString);
   if (m_pDatabase == 0)
   {
      // there was an error and the database could not
      // be opened.
      //Con::executef(this, "onOpenFailed()", m_szErrorString);
      return false;
   }
   else
   {
      // database was opened without error
      //Con::executef(this, "onOpened()");//no namespace onOpened...?
   }
   return true;
}

int SQLiteObject::ExecuteSQL(const char* sql)
{
   int iResult;
   sqlite_resultset* pResultSet;

   if (gConsole)
		gConsole->addOutput(sql);
   //Con::printf("SQLite QUERY: %s",sql);//Ecstasy Motion, debug.

   // create a new resultset
   pResultSet = new sqlite_resultset;

   if (pResultSet)
   {
      pResultSet->bValid = false;
      pResultSet->iCurrentColumn = 0;
      pResultSet->iCurrentRow = 0;
      pResultSet->iNumCols = 0;
      pResultSet->iNumRows = 0;
      pResultSet->iResultSet = m_iNextResultSet;
      pResultSet->vRows.clear();
      m_iLastResultSet = m_iNextResultSet;
      m_iNextResultSet++;
   }
   else
      return 0;

   iResult = sqlite_exec(m_pDatabase, sql, Callback, (void*)pResultSet, &m_szErrorString);
   if (iResult == 0)
   {
      //SQLITE_OK
      SaveResultSet(pResultSet);
      //Con::executef(this, "onQueryFinished()");//hm, no namespace onQueryFinished... ?
      return pResultSet->iResultSet;
   }
   else
   {
      // error occured
      //Con::executef(this, "onQueryFailed", m_szErrorString);
      delete pResultSet;
      return 0;
   }

   return 0;
}

void SQLiteObject::CloseDatabase()
{
   if (m_pDatabase)
      sqlite_close(m_pDatabase);

   m_pDatabase = NULL;
}

void SQLiteObject::NextRow(int resultSet)
{
   sqlite_resultset* pResultSet;

   pResultSet = GetResultSet(resultSet);
   if (!pResultSet)
      return;

   pResultSet->iCurrentRow++;
}

bool SQLiteObject::EndOfResult(int resultSet)
{
   sqlite_resultset* pResultSet;

   pResultSet = GetResultSet(resultSet);
   if (!pResultSet)
      return true;

   if (pResultSet->iCurrentRow >= pResultSet->iNumRows)
      return true;
   
   return false;
}

void SQLiteObject::ClearErrorString()
{
   if (m_szErrorString)
      sqlite_freemem(m_szErrorString);

   m_szErrorString = NULL;
}

void SQLiteObject::ClearResultSet(int index)
{
   sqlite_resultset* resultSet;
   sqlite_resultrow* resultRow;
   int rows, cols, iResultSet;
   
   // Get the result set specified by index
   resultSet = GetResultSet(index);
   iResultSet = GetResultSetIndex(index);
   if ((!resultSet) || (!resultSet->bValid))
   {
      //Con::warnf("Warning SQLiteObject::ClearResultSet(%i) failed to retrieve specified result set.  Result set was NOT cleared.", index);
      return;
   }
   // Now we have the specific result set to be cleared.
   // What we need to do now is iterate through each "Column" in each "Row"
   // and free the strings, then delete the entries.
	std::vector<sqlite_resultrow*>::iterator iRow;
   std::vector<char*>::iterator iColumnName;
   std::vector<char*>::iterator iColumnValue;

   for (iRow = resultSet->vRows.begin(); iRow != resultSet->vRows.end(); iRow++)
   {
      // Iterate through rows
      // for each row iterate through all the column values and names
      for (iColumnName = (*iRow)->vColumnNames.begin(); iColumnName != (*iRow)->vColumnNames.end(); iColumnName++)
      {
         // Iterate through column names.  Free the memory.
         delete[] (*iColumnName);
      }
      for (iColumnValue = (*iRow)->vColumnValues.begin(); iColumnValue != (*iRow)->vColumnValues.end(); iColumnValue++)
      {
         // Iterate through column values.  Free the memory.
         delete[] (*iColumnValue);
      }
      // free memory used by the row
      delete (*iRow);
   }
   // empty the resultset
   resultSet->vRows.clear();
   resultSet->bValid = false;
   delete resultSet;
	
	//HMM, can't erase_fast or erase(), do I still need to do this somehow?
   //m_vResultSets.erase(iResultSet);
}

sqlite_resultset* SQLiteObject::GetResultSet(int iResultSet)
{
   // Get the result set specified by iResultSet
   std::vector<sqlite_resultset*>::iterator i;
   for (i = m_vResultSets.begin(); i != m_vResultSets.end(); i++)
   {
      if ((*i)->iResultSet == iResultSet)
         break;
   }

   return *i;
}

int SQLiteObject::GetResultSetIndex(int iResultSet)
{
   int iIndex;
   // Get the result set specified by iResultSet
   std::vector<sqlite_resultset*>::iterator i;
   iIndex = 0;
   for (i = m_vResultSets.begin(); i != m_vResultSets.end(); i++)
   {
      if ((*i)->iResultSet == iResultSet)
         break;
      iIndex++;
   }

   return iIndex;
}

bool SQLiteObject::SaveResultSet(sqlite_resultset* pResultSet)
{
   // Basically just add this to our vector.  It should already be filled up.
   pResultSet->bValid = true;
   m_vResultSets.push_back(pResultSet);

   return true;
}

int SQLiteObject::GetColumnIndex(int iResult, const char* columnName)
{
   int iIndex;
   std::vector<char*>::iterator i;
   sqlite_resultset* pResultSet;
   sqlite_resultrow* pRow;

   pResultSet = GetResultSet(iResult);
   if (!pResultSet)
      return 0;

   pRow = pResultSet->vRows[0];
   if (!pRow)
      return 0;
   
   iIndex = 0;
   for (i = pRow->vColumnNames.begin(); i != pRow->vColumnNames.end(); i++)
   {
      if (stricmp((*i), columnName) == 0)
         return iIndex + 1;
      iIndex++;
   }

   return 0;
}


// TO DO: SQLiteObject needs to be tied in to AngelScript...
//-----------------------------------------------------------------------
// These functions are the code that actually tie our object into the scripting
// language.  As you can see each one of these is called by scrpit and in turn
// calls the C++ class function.
//ConsoleMethod(SQLiteObject, openDatabase, bool, 3, 3, "(const char* filename) Opens the database specifed by filename.  Returns true or false.")
//{
//   return object->OpenDatabase(argv[2]);
//}
//
//ConsoleMethod(SQLiteObject, closeDatabase, void, 2, 2, "Closes the active database.")
//{
//   object->CloseDatabase();
//}
//
//ConsoleMethod(SQLiteObject, query, int, 4, 0, "(const char* sql, int mode) Performs an SQL query on the open database and returns an identifier to a valid result set. mode is currently unused, and is reserved for future use.")
//{
//   int iCount;
//   int iIndex, iLen, iNewIndex, iArg, iArgLen, i;
//   char* szNew;
//
//   if (argc == 4)
//      return object->ExecuteSQL(argv[2]);
//   else if (argc > 4)
//   {
//      // Support for printf type querys, as per Ben Garney's suggestion
//      // Basically what this does is allow the user to insert questino marks into thier query that will
//      // be replaced with actual data.  For example:
//      // "SELECT * FROM data WHERE id=? AND age<7 AND name LIKE ?"
//
//      // scan the query and count the question marks
//      iCount = 0;
//      iLen = dStrlen(argv[2]);
//      for (iIndex = 0; iIndex < iLen; iIndex++)         
//      {
//         if (argv[2][iIndex] == '?')
//            iCount++;
//      }
//      
//      // now that we know how many replacements we have, we need to make sure we
//      // have enough arguments to replace them all.  All arguments above 4 should be our data
//      if (argc - 4 == iCount)
//      {
//         // ok we have the correct number of arguments
//         // so now we need to calc the length of the new query string.  This is easily achieved.
//         // We simply take our base string length, subtract the question marks, then add in
//         // the number of total characters used by our arguments.
//         iLen = dStrlen(argv[2]) - iCount;
//         for (iIndex = 1; iIndex <= iCount; iIndex++)
//         {
//            iLen = iLen + dStrlen(argv[iIndex+3]);
//         }
//         // iLen should now be the length of our new string
//         szNew = new char[iLen];
//
//         // now we need to replace all the question marks with the actual arguments
//         iLen = dStrlen(argv[2]);
//         iNewIndex = 0;
//         iArg = 1;
//         for (iIndex = 0; iIndex <= iLen; iIndex++)
//         {
//            if (argv[2][iIndex] == '?')
//            {
//               // ok we need to replace this question mark with the actual argument
//               // and iterate our pointers and everything as needed.  This is no doubt
//               // not the best way to do this, but it works for me for now.
//               // My god this is really a mess.
//               iArgLen = dStrlen(argv[iArg + 3]);
//               // copy first character
//               szNew[iNewIndex] = argv[iArg + 3][0];
//               // copy rest of characters, and increment iNewIndex
//               for (i = 1; i < iArgLen; i++)
//               {
//                  iNewIndex++;
//                  szNew[iNewIndex] = argv[iArg + 3][i];
//               }
//               iArg++;
//
//            }
//            else
//               szNew[iNewIndex] = argv[2][iIndex];
//
//            iNewIndex++;
//         }
//      }
//      else
//         return 0; // incorrect number of question marks vs arguments
//      Con::printf("Old SQL: %s\nNew SQL: %s", argv[2], szNew);
//      return object->ExecuteSQL(szNew);
//   }
//
//   return 0;
//}
//
//ConsoleMethod(SQLiteObject, clearResult, void, 3, 3, "(int resultSet) Clears memory used by the specified result set, and deletes the result set.")
//{
//   object->ClearResultSet(dAtoi(argv[2]));
//}
//
//ConsoleMethod(SQLiteObject, nextRow, void, 3, 3, "(int resultSet) Moves the result set's row pointer to the next row.")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pResultSet->iCurrentRow++;
//   }
//}
//
//ConsoleMethod(SQLiteObject, previousRow, void, 3, 3, "(int resultSet) Moves the result set's row pointer to the previous row")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pResultSet->iCurrentRow--;
//   }
//}
//
//ConsoleMethod(SQLiteObject, firstRow, void, 3, 3, "(int resultSet) Moves the result set's row pointer to the very first row in the result set.")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pResultSet->iCurrentRow = 0;
//   }
//}
//
//ConsoleMethod(SQLiteObject, lastRow, void, 3, 3, "(int resultSet) Moves the result set's row pointer to the very last row in the result set.")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pResultSet->iCurrentRow = pResultSet->iNumRows - 1;
//   }
//}
//
//ConsoleMethod(SQLiteObject, setRow, void, 4, 4, "(int resultSet int row) Moves the result set's row pointer to the row specified.  Row indices start at 1 not 0.")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pResultSet->iCurrentRow = dAtoi(argv[3]) - 1;
//   }
//}
//
//ConsoleMethod(SQLiteObject, getRow, int, 3, 3, "(int resultSet) Returns what row the result set's row pointer is currently on.")
//{
//   sqlite_resultset* pResultSet;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      return pResultSet->iCurrentRow + 1;
//   }
//   else
//      return 0;
//}
//
//ConsoleMethod(SQLiteObject, numRows, int, 3, 3, "(int resultSet) Returns the number of rows in the result set.")
//{
//   sqlite_resultset* pResultSet = NULL;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      return pResultSet->iNumRows;
//   }
//   else
//      return 0;
//}
//
//ConsoleMethod(SQLiteObject, numColumns, int, 3, 3, "(int resultSet) Returns the number of columns in the result set.")
//{
//   sqlite_resultset* pResultSet = NULL;
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      return pResultSet->iNumCols;
//   }
//   else
//      return 0;
//}
//
//ConsoleMethod(SQLiteObject, endOfResult, bool, 3, 3, "(int resultSet) Checks to see if the internal pointer for the specified result set is at the end, indicating there are no more rows left to read.")
//{
//   return object->EndOfResult(dAtoi(argv[2]));
//}
//
//ConsoleMethod(SQLiteObject, EOR, bool, 3, 3, "(int resultSet) Same as endOfResult().")
//{
//   return object->EndOfResult(dAtoi(argv[2]));
//}
//
//ConsoleMethod(SQLiteObject, EOF, bool, 3, 3, "(int resultSet) Same as endOfResult().")
//{
//   return object->EndOfResult(dAtoi(argv[2]));
//}
//
//ConsoleMethod(SQLiteObject, getColumnIndex, int, 4, 4, "(resultSet columnName) Looks up the specified column name in the specified result set, and returns the columns index number.  A return value of 0 indicates the lookup failed for some reason (usually this indicates you specified a column name that doesn't exist or is spelled wrong).")
//{
//   return object->GetColumnIndex(dAtoi(argv[2]), argv[3]);
//}
//
//ConsoleMethod(SQLiteObject, getColumnName, const char *, 4, 4, "(resultSet columnIndex) Looks up the specified column index in the specified result set, and returns the column's name.  A return value of an empty string indicates the lookup failed for some reason (usually this indicates you specified a column index that is invalid or exceeds the number of columns in the result set). Columns are index starting with 1 not 0")
//{
//   sqlite_resultset* pResultSet;
//   sqlite_resultrow* pRow;
//   VectorPtr<char*>::iterator iName;
//   VectorPtr<char*>::iterator iValue;
//   int iColumn;
//
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pRow = pResultSet->vRows[pResultSet->iCurrentRow];
//      if (!pRow)
//         return "";
//      
//      // We assume they specified column by index.  If they know the column name they wouldn't be calling this function :)
//      iColumn = dAtoi(argv[3]);
//      if (iColumn == 0)
//         return "";  // column indices start at 1, not 0
//
//      // now we should have an index for our column name
//      if (pRow->vColumnNames[iColumn])
//         return pRow->vColumnNames[iColumn];
//      else
//         return "";
//   }
//   else
//      return "";
//}
//
//ConsoleMethod(SQLiteObject, getColumn, const char *, 4, 4, "(resultSet column) Returns the value of the specified column (Column can be specified by name or index) in the current row of the specified result set. If the call fails, the returned string will indicate the error.")
//{
//   sqlite_resultset* pResultSet;
//   sqlite_resultrow* pRow;
//   VectorPtr<char*>::iterator iName;
//   VectorPtr<char*>::iterator iValue;
//   int iColumn;
//
//   pResultSet = object->GetResultSet(dAtoi(argv[2]));
//   if (pResultSet)
//   {
//      pRow = pResultSet->vRows[pResultSet->iCurrentRow];
//      if (!pRow)
//         return "invalid_row";
//      
//      // Is column specified by a name or an index?
//      iColumn = dAtoi(argv[3]);
//      if (iColumn == 0)
//      {
//         // column was specified by a name
//         iColumn = object->GetColumnIndex(dAtoi(argv[2]), argv[3]);
//         // if this is still 0 then we have some error
//         if (iColumn == 0)
//            return "invalid_column";
//      }
//
//      // We temporarily padded the index in GetColumnIndex() so we could return a 
//      // 0 for error.  So now we need to drop it back down.
//      iColumn--;
//
//      // now we should have an index for our column data
//      if (pRow->vColumnValues[iColumn])
//         return pRow->vColumnValues[iColumn];
//      else
//         return "NULL";
//   }
//   else
//      return "invalid_result_set";
//}
//
//ConsoleMethod(SQLiteObject, escapeString, const char *, 3, 3, "(string) Escapes the given string, making it safer to pass into a query.")
//{
//   // essentially what we need to do here is scan the string for any occurances of: ', ", and \
//   // and prepend them with a slash: \', \", \\
//
//   // to do this we first need to know how many characters we are replacing so we can calculate
//   // the size of the new string
//   int iCount;
//   int iIndex, iLen, iNewIndex;
//   char* szNew;
//
//   iCount = 0;
//   iLen = dStrlen(argv[2]);
//   for (iIndex = 0; iIndex < iLen; iIndex++)         
//   {
//      if (argv[2][iIndex] == '\'')
//         iCount++;
//      else if (argv[2][iIndex] == '\"')
//         iCount++;
//      else if (argv[2][iIndex] == '\\')
//         iCount++;
//
//   }
////   Con::printf("escapeString counts %i instances of characters to be escaped.  New string will be %i characters longer for a total of %i characters.", iCount, iCount, iLen+iCount);
//   szNew = new char[iLen+iCount];
//   iNewIndex = 0;
//   for (iIndex = 0; iIndex <= iLen; iIndex++)         
//   {
//      if (argv[2][iIndex] == '\'')
//      {
//         szNew[iNewIndex] = '\\';
//         iNewIndex++;
//         szNew[iNewIndex] = '\'';
//      }
//      else if (argv[2][iIndex] == '\"')
//      {
//         szNew[iNewIndex] = '\\';
//         iNewIndex++;
//         szNew[iNewIndex] = '\"';
//      }
//      else if (argv[2][iIndex] == '\\')
//      {
//         szNew[iNewIndex] = '\\';
//         iNewIndex++;
//         szNew[iNewIndex] = '\\';
//      }
//      else
//         szNew[iNewIndex] = argv[2][iIndex];
//
//      iNewIndex++;
//   }
////   Con::printf("Last characters of each string (new, old): %s, %s", argv[2][iIndex-1], szNew[iNewIndex-1]);
////   Con::printf("Old String: %s\nNew String: %s", argv[2], szNew);
//
//   return szNew;
//}
