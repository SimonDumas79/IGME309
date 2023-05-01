#include "Octant.h"
using namespace BTX;
//  Octant
uint Octant::m_uOctantCount = 0;
uint Octant::m_uMaxLevel = 3;
uint Octant::m_uIdealEntityCount = 5;

Octant::Octant(uint a_nMaxLevel, uint a_nIdealEntityCount)
{
	/*
	* This constructor is meant to be used ONLY on the root node, there is already a working constructor
	* that will take a size and a center to create a new space
	*/
	Init();//Init the default values
	m_uOctantCount = 0;
	m_uMaxLevel = a_nMaxLevel;
	m_uIdealEntityCount = a_nIdealEntityCount;
	m_uID = m_uOctantCount;

	m_pRoot = this;
	m_lChild.clear();

	//set the size and shape of the octree
	std::vector<vector3> lMinMax;

	//Get how many objects there are in the world
	uint nObjects = m_pEntityMngr->GetEntityCount();

	for (uint i = 0; i < nObjects; i++)
	{
		Entity* entity = m_pEntityMngr->GetEntity(i);
		RigidBody* rigidBody = entity->GetRigidBody();
		lMinMax.push_back(rigidBody->GetMinGlobal());
		lMinMax.push_back(rigidBody->GetMaxGlobal());
	}

	RigidBody* rigidBody = new RigidBody(lMinMax);

	vector3 halfWidth = rigidBody->GetHalfWidth();
	float max = halfWidth.x;

	for (int i = 1; i < 3; i++)
	{
		if (max < halfWidth[i])
		{
			max = halfWidth[i];
		}
	}

	vector3 center = rigidBody->GetCenterLocal();
	lMinMax.clear();
	SafeDelete(rigidBody);

	m_fSize = max * 2.0f;
	m_v3Center = center;
	m_v3Min = m_v3Center - (vector3(max));
	m_v3Max = m_v3Center + (vector3(max));


	m_uOctantCount++; //When we add an octant we increment the count
	ConstructTree(m_uMaxLevel); //Construct the children
}
bool Octant::CheckCollision(vector3 min1, vector3 max1, vector3 min2, vector3 max2)
{
	if (max1.x < min2.x || min1.x > max2.x) 
		return false;

	if (max1.y < min2.y || min1.y > max2.y) 
		return false;

	if (max1.z < min2.z || min1.z > max2.z) 
		return false;

	return true;
}
bool Octant::IsColliding(uint a_uRBIndex)
{
	uint nObjects = m_pEntityMngr->GetEntityCount();
	//If the index given is larger than the number of elements in the bounding object there is no collision
	if (nObjects < a_uRBIndex)
	{
		return false;
	}

	//Get how many objects there are in the world
	if (m_pParent != nullptr)
	{
		nObjects = m_pParent->m_EntityList.size();
	}
	for (uint i = 0; i < m_EntityList.size(); i++)
	{
		Entity* current = m_pEntityMngr->GetEntity(i);
		RigidBody* rigidBody = current->GetRigidBody();

		if (CheckCollision(m_v3Min, m_v3Max,
			rigidBody->GetMinGlobal(), rigidBody->GetMaxGlobal()))
		{
			m_EntityList.push_back(i);
			return true;
		}
	}
	return false; // for the sake of startup code
}
void Octant::Display(uint a_nIndex, vector3 a_v3Color)
{
	// Display the specified octant 
	if(m_lChild.size()>0)
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_lChild[a_nIndex]->GetCenterGlobal()) *
		glm::scale(vector3(m_lChild[a_nIndex]->GetSize())), a_v3Color);
}
void Octant::Display(vector3 a_v3Color)
{
	//this is meant to be a recursive method, in starter code will only display the root
	//even if other objects are created
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) *
		glm::scale(vector3(m_fSize)), a_v3Color);
}
void Octant::Subdivide(void)
{
	//If this node has reach the maximum depth return without changes
	if (m_uLevel >= m_uMaxLevel)
		return;

	//If this node has been already subdivided return without changes
	if (m_uChildren != 0)
		return;

	float x1 = m_v3Center.x - m_fSize / 4;
	float x2 = m_v3Center.x + m_fSize / 4;

	float y1 = m_v3Center.y - m_fSize / 4;
	float y2 = m_v3Center.y + m_fSize / 4;

	float z1 = m_v3Center.z - m_fSize / 4;
	float z2 = m_v3Center.z + m_fSize / 4;

	vector3 octants[8];

	octants[0] = vector3(x1, y1, z1);
	octants[1] = vector3(x2, y1, z1);
	octants[2] = vector3(x1, y2, z1);
	octants[3] = vector3(x2, y2, z1);
	octants[4] = vector3(x1, y1, z2);
	octants[5] = vector3(x2, y1, z2);
	octants[6] = vector3(x1, y2, z2);
	octants[7] = vector3(x2, y2, z2);


	for (int i = 0; i < 8; i++)
	{
		m_pChild[i] = new Octant(octants[i], m_fSize / 2);
		m_pChild[i]->m_pParent = this;
		m_uOctantCount++;
	}
	m_uChildren = 8;
}
bool Octant::ContainsAtLeast(uint a_nEntities)
{
	//if it isnt the root, get the parents number otherwise get the total number of entities
	int nObjects;
	if (m_pParent != nullptr)
	{
		nObjects = m_pParent->m_EntityList.size();
	}
	else 
	{
		nObjects = m_pEntityMngr->GetEntityCount();
	}

	int contained = 0;
	//loop through all of the entities that can be contained in this octant
	for (uint i = 0; i < nObjects; i++)
	{
		Entity* entity = m_pEntityMngr->GetEntity(i);
		if (m_pParent != nullptr)
		{
			Entity* entity = m_pEntityMngr->GetEntity(m_pParent->m_EntityList[i]);
		}
		
		if (m_pEntityMngr->IsInDimension(m_pEntityMngr->GetUniqueID(i), m_uID))
		{
			contained++;
		}
	}

	return contained > a_nEntities;
}
void Octant::AssignIDtoEntity(void)
{
	//Recursive method
	//Have to traverse the tree and make sure to tell the entity manager
	//what octant (space) each object is at

	for (uint i = 0; i < m_pEntityMngr->GetEntityCount(); i++)
	{
		Entity* entity = m_pEntityMngr->GetEntity(i);
		RigidBody* rigidBody = entity->GetRigidBody();
		if (CheckCollision(m_v3Min, m_v3Max, rigidBody->GetMinGlobal(), rigidBody->GetMaxGlobal()))
		{
			m_EntityList.push_back(i);
			m_pEntityMngr->AddDimension(i, m_uID);
			if (!IsLeaf())
			{
				m_pChild[i]->AssignIDtoEntity();
			}
		}
				
	}

}
//-------------------------------------------------------------------------------------------------------------------
// You can assume the following is fine and does not need changes, you may add onto it but the code is fine as is
// in the proposed solution.
void Octant::Init(void)
{
	m_uChildren = 0;

	m_fSize = 0.0f;

	m_uID = m_uOctantCount;
	m_uLevel = 0;

	m_v3Center = vector3(0.0f);
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	m_pRoot = nullptr;
	m_pParent = nullptr;
	for (uint n = 0; n < 8; n++)
	{
		m_pChild[n] = nullptr;
	}
}
void Octant::Swap(Octant& other)
{
	std::swap(m_uChildren, other.m_uChildren);

	std::swap(m_fSize, other.m_fSize);
	std::swap(m_uID, other.m_uID);
	std::swap(m_pRoot, other.m_pRoot);
	std::swap(m_lChild, other.m_lChild);

	std::swap(m_v3Center, other.m_v3Center);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	std::swap(m_uLevel, other.m_uLevel);
	std::swap(m_pParent, other.m_pParent);
	for (uint i = 0; i < 8; i++)
	{
		std::swap(m_pChild[i], other.m_pChild[i]);
	}
}
void Octant::Release(void)
{
	//this is a special kind of release, it will only happen for the root
	if (m_uLevel == 0)
	{
		KillBranches();
	}
	m_uChildren = 0;
	m_fSize = 0.0f;
	m_EntityList.clear();
	m_lChild.clear();
}
void Octant::ConstructTree(uint a_nMaxLevel)
{
	//If this method is tried to be applied to something else
	//other than the root, don't.
	if (m_uLevel != 0)
		return;

	m_uMaxLevel = a_nMaxLevel; //Make sure you mark which is the maximum level you are willing to reach
	m_uOctantCount = 1;// We will always start with one octant

	//If this was initialized before make sure its empty
	m_EntityList.clear();//Make sure the list of entities inside this octant is empty
	KillBranches();
	m_lChild.clear();

	//If we have more entities than those that we ideally want in this octant we subdivide it
	if (ContainsAtLeast(m_uIdealEntityCount))
	{
		Subdivide();
	}
	AssignIDtoEntity();//Add octant ID to Entities
	ConstructList();//construct the list of objects
}
//The big 3
Octant::Octant(vector3 a_v3Center, float a_fSize)
{
	//Will create the octant object based on the center and size but will not construct children
	Init();
	m_v3Center = a_v3Center;
	m_fSize = a_fSize;

	m_v3Min = m_v3Center - (vector3(m_fSize) / 2.0f);
	m_v3Max = m_v3Center + (vector3(m_fSize) / 2.0f);

	m_uOctantCount++;
}
Octant::Octant(Octant const& other)
{
	m_uChildren = other.m_uChildren;
	m_v3Center = other.m_v3Center;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;

	m_fSize = other.m_fSize;
	m_uID = other.m_uID;
	m_uLevel = other.m_uLevel;
	m_pParent = other.m_pParent;

	m_pRoot, other.m_pRoot;
	m_lChild, other.m_lChild;

	m_pModelMngr = ModelManager::GetInstance();
	m_pEntityMngr = EntityManager::GetInstance();

	for (uint i = 0; i < 8; i++)
	{
		m_pChild[i] = other.m_pChild[i];
	}
}
Octant& Octant::operator=(Octant const& other)
{
	if (this != &other)
	{
		Release();
		Init();
		Octant temp(other);
		Swap(temp);
	}
	return *this;
}
Octant::~Octant() { Release(); };
//Accessors
float Octant::GetSize(void) { return m_fSize; }
vector3 Octant::GetCenterGlobal(void) { return m_v3Center; }
vector3 Octant::GetMinGlobal(void) { return m_v3Min; }
vector3 Octant::GetMaxGlobal(void) { return m_v3Max; }
uint Octant::GetOctantCount(void) { return m_uOctantCount; }
bool Octant::IsLeaf(void) { return m_uChildren == 0; }
Octant* Octant::GetParent(void) { return m_pParent; }
//--- other methods
Octant * Octant::GetChild(uint a_nChild)
{
	//if its asking for more than the 8th children return nullptr, as we don't have one
	if (a_nChild > 7) return nullptr;
	return m_pChild[a_nChild];
}
void Octant::KillBranches(void)
{
	/*This method has recursive behavior that is somewhat hard to explain line by line but
	it will traverse the whole tree until it reaches a node with no children and
	once it returns from it to its parent it will kill all of its children, recursively until
	it reaches back the node that called it.*/
	for (uint nIndex = 0; nIndex < m_uChildren; nIndex++)
	{
		m_pChild[nIndex]->KillBranches();
		delete m_pChild[nIndex];
		m_pChild[nIndex] = nullptr;
	}
	m_uChildren = 0;
}
void Octant::DisplayLeaves(vector3 a_v3Color)
{
	/*
	* Recursive method
	* it will traverse the tree until it find leaves and once found will draw them
	*/
	uint nLeafs = m_lChild.size(); //get how many children this will have (either 0 or 8)
	for (uint nChild = 0; nChild < nLeafs; nChild++)
	{
		m_lChild[nChild]->DisplayLeaves(a_v3Color);
	}
	//Draw the cube
	m_pModelMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) *
		glm::scale(vector3(m_fSize)), a_v3Color);
}
void Octant::ClearEntityList(void)
{
	/*
	* Recursive method
	* It will traverse the tree until it find leaves and once found will clear its content
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ClearEntityList();
	}
	m_EntityList.clear();
}
void Octant::ConstructList(void)
{
	/*
	* Recursive method
	* It will traverse the tree adding children
	*/
	for (uint nChild = 0; nChild < m_uChildren; nChild++)
	{
		m_pChild[nChild]->ConstructList();
	}
	//if we find a non-empty child add it to the tree
	if (m_EntityList.size() > 0)
	{
		m_pRoot->m_lChild.push_back(this);
	}
}
