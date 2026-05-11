// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB). This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "VehicleImporter.h"
#include "USDImporterWidget.h"
#include "CarlaTools.h"

#include <util/ue-header-guard-begin.h>
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Common/TcpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Async/Async.h"
#include "Containers/Ticker.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Factories/FbxImportUI.h"
#include "AssetImportTask.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "EditorAssetLibrary.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "ChaosVehicleWheel.h"
#include "Factories/BlueprintFactory.h"
#include "Engine/Blueprint.h"
#include <util/ue-header-guard-end.h>


static constexpr int32 GImporterPort = 18583;

static constexpr int32 GMaxMessageBytes = 10 * 1024 * 1024;





FVehicleImporterServer* UVehicleImporter::ServerRunnable = nullptr;
FRunnableThread*        UVehicleImporter::ServerThread   = nullptr;

void UVehicleImporter::StartServer()
{
  if (ServerRunnable)
    return;

  ServerRunnable = new FVehicleImporterServer();
  ServerThread   = FRunnableThread::Create(ServerRunnable,
                     TEXT("CarlaVehicleImporter"),
                     0, TPri_BelowNormal);
  UE_LOG(LogCarlaTools, Log, TEXT("VehicleImporter: listening on port %d"), GImporterPort);
}

void UVehicleImporter::StopServer()
{
  if (ServerRunnable)
    ServerRunnable->Stop();

  if (ServerThread)
  {
    ServerThread->WaitForCompletion();
    delete ServerThread;
    ServerThread = nullptr;
  }
  delete ServerRunnable;
  ServerRunnable = nullptr;
}





FVehicleImporterServer::FVehicleImporterServer()  = default;
FVehicleImporterServer::~FVehicleImporterServer() = default;

bool FVehicleImporterServer::Init()
{
  ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
  if (!SS)
    return false;

  ListenSocket = FTcpSocketBuilder(TEXT("CarlaVehicleImporterListen"))
    .AsReusable()
    .BoundToEndpoint(FIPv4Endpoint(FIPv4Address::Any, GImporterPort))
    .Listening(1)
    .Build();

  if (!ListenSocket)
  {
    UE_LOG(LogCarlaTools, Error,
           TEXT("VehicleImporter: failed to bind port %d"), GImporterPort);
    return false;
  }

  bRunning = true;
  return true;
}

uint32 FVehicleImporterServer::Run()
{
  while (bRunning)
  {
    bool bPending = false;
    if (ListenSocket->WaitForPendingConnection(bPending, FTimespan::FromSeconds(1.0)))
    {
      if (bPending)
      {
        FSocket* Client = ListenSocket->Accept(TEXT("CarlaStudio"));
        if (Client)
        {
          ServeClient(Client);
          Client->Close();
          ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Client);
        }
      }
    }
  }
  return 0;
}

void FVehicleImporterServer::Stop()
{
  bRunning = false;
  if (ListenSocket)
  {
    ListenSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
    ListenSocket = nullptr;
  }
}





static bool RecvAll(FSocket* S, uint8* Buf, int32 Len)
{
  int32 Received = 0;
  while (Received < Len)
  {
    int32 Got = 0;
    if (!S->Recv(Buf + Received, Len - Received, Got) || Got <= 0)
      return false;
    Received += Got;
  }
  return true;
}

static bool SendAll(FSocket* S, const uint8* Buf, int32 Len)
{
  int32 Sent = 0;
  while (Sent < Len)
  {
    int32 Written = 0;
    if (!S->Send(Buf + Sent, Len - Sent, Written) || Written <= 0)
      return false;
    Sent += Written;
  }
  return true;
}

void FVehicleImporterServer::ServeClient(FSocket* Client)
{

  uint8 LenBuf[4];
  if (!RecvAll(Client, LenBuf, 4))
    return;

  const int32 MsgLen = (int32)(LenBuf[0]
    | ((uint32)LenBuf[1] << 8)
    | ((uint32)LenBuf[2] << 16)
    | ((uint32)LenBuf[3] << 24));

  if (MsgLen <= 0 || MsgLen > GMaxMessageBytes)
  {
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VehicleImporter: bad message length %d"), MsgLen);
    return;
  }

  TArray<uint8> Body;
  Body.SetNumUninitialized(MsgLen);
  if (!RecvAll(Client, Body.GetData(), MsgLen))
    return;

  const FString JsonStr = FString(UTF8_TO_TCHAR(
    reinterpret_cast<const ANSICHAR*>(Body.GetData())));






  FString Response;
  TSharedPtr<FJsonObject> Root;
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonStr);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
  {
    Response = MakeResponse(false, TEXT(""), TEXT("JSON parse error"));
  }
  else
  {
    FString Action;
    Root->TryGetStringField(TEXT("action"), Action);
    if (Action.Equals(TEXT("spawn"), ESearchCase::IgnoreCase))
    {
      FSpawnRequest Req;
      Root->TryGetStringField(TEXT("asset_path"), Req.AssetPath);
      double V = 0.0;
      if (Root->TryGetNumberField(TEXT("x"),   V)) Req.Loc.X   = (float)V;
      if (Root->TryGetNumberField(TEXT("y"),   V)) Req.Loc.Y   = (float)V;
      if (Root->TryGetNumberField(TEXT("z"),   V)) Req.Loc.Z   = (float)V;
      if (Root->TryGetNumberField(TEXT("yaw"), V)) Req.Yaw     = (float)V;

      auto P = MakeShared<TPromise<FString>>();
      TFuture<FString> Future = P->GetFuture();
      FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
        [this, Req, P](float) -> bool {
          P->SetValue(ProcessSpawn(Req));
          return false;
        }));
      Response = Future.Get();
    }
    else
    {
      FVehicleImportSpec Spec;
      if (!ParseSpec(JsonStr, Spec))
      {
        Response = MakeResponse(false, TEXT(""), TEXT("JSON parse error"));
      }
      else
      {
        auto P = MakeShared<TPromise<FString>>();
        TFuture<FString> Future = P->GetFuture();
        FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
          [this, Spec, P](float) -> bool {
            P->SetValue(ProcessSpec(Spec));
            return false;
          }));
        Response = Future.Get();
      }
    }
  }


  FTCHARToUTF8 ResponseUTF8(*Response);
  const int32 RespLen = ResponseUTF8.Length();
  uint8 RespLenBuf[4] = {
    (uint8)(RespLen & 0xFF),
    (uint8)((RespLen >> 8)  & 0xFF),
    (uint8)((RespLen >> 16) & 0xFF),
    (uint8)((RespLen >> 24) & 0xFF)
  };
  SendAll(Client, RespLenBuf, 4);
  SendAll(Client, reinterpret_cast<const uint8*>(ResponseUTF8.Get()), RespLen);
}





static float JF(const TSharedPtr<FJsonObject>& O, const FString& K, float Def = 0.f)
{
  double V = Def;
  O->TryGetNumberField(K, V);
  return (float)V;
}

static TSharedPtr<FJsonObject> JO(const TSharedPtr<FJsonObject>& O, const FString& K)
{
  const TSharedPtr<FJsonObject>* Sub = nullptr;
  if (O->TryGetObjectField(K, Sub))
    return *Sub;
  return nullptr;
}

static void ParseWheel(const TSharedPtr<FJsonObject>& Root,
                       const FString& Key,
                       FWheelImportSpec& Out)
{
  TSharedPtr<FJsonObject> W = JO(Root, Key);
  if (!W) return;
  Out.X              = JF(W, "x");
  Out.Y              = JF(W, "y");
  Out.Z              = JF(W, "z");
  Out.Radius         = JF(W, "radius",           33.f);
  Out.MaxSteerAngle  = JF(W, "max_steer_angle",  70.f);
  Out.MaxBrakeTorque = JF(W, "max_brake_torque", 1500.f);
  Out.SuspMaxRaise   = JF(W, "susp_max_raise",   10.f);
  Out.SuspMaxDrop    = JF(W, "susp_max_drop",    10.f);
}

bool FVehicleImporterServer::ParseSpec(const FString& Json, FVehicleImportSpec& Out)
{
  TSharedPtr<FJsonObject> Root;
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    return false;

  Root->TryGetStringField(TEXT("vehicle_name"),   Out.VehicleName);
  Root->TryGetStringField(TEXT("mesh_path"),       Out.MeshFilePath);
  Root->TryGetStringField(TEXT("content_path"),    Out.ContentPath);
  Root->TryGetStringField(TEXT("base_vehicle_bp"), Out.BaseVehicleBP);

  double V = 1500.0;
  Root->TryGetNumberField(TEXT("mass"),            V); Out.Mass = (float)V;
  Root->TryGetNumberField(TEXT("susp_damping"),    V); Out.SuspDamping = (float)V;

  V = 1.0;
  Root->TryGetNumberField(TEXT("source_scale_to_cm"), V); Out.SourceScaleToCm = (float)V;
  int32 IV = 2;
  Root->TryGetNumberField(TEXT("source_up_axis"),      IV); Out.SourceUpAxis      = IV;
  IV = 0;
  Root->TryGetNumberField(TEXT("source_forward_axis"), IV); Out.SourceForwardAxis = IV;

  ParseWheel(Root, TEXT("wheel_fl"), Out.WheelFL);
  ParseWheel(Root, TEXT("wheel_fr"), Out.WheelFR);
  ParseWheel(Root, TEXT("wheel_rl"), Out.WheelRL);
  ParseWheel(Root, TEXT("wheel_rr"), Out.WheelRR);

  return !Out.VehicleName.IsEmpty() && !Out.MeshFilePath.IsEmpty();
}

FString FVehicleImporterServer::MakeResponse(bool bOk, const FString& Path, const FString& Err)
{
  TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
  Obj->SetBoolField(TEXT("success"),      bOk);
  Obj->SetStringField(TEXT("asset_path"), Path);
  Obj->SetStringField(TEXT("error"),      Err);

  FString Out;
  TSharedRef<TJsonWriter<>> W = TJsonWriterFactory<>::Create(&Out);
  FJsonSerializer::Serialize(Obj.ToSharedRef(), W);
  return Out;
}








static FString SanitizeAssetName(const FString& In)
{
  FString Out;
  Out.Reserve(In.Len());
  for (TCHAR C : In)
  {
    const bool bOk = (C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z')
                  || (C >= '0' && C <= '9') || C == '_';
    Out.AppendChar(bOk ? C : TEXT('_'));
  }
  if (Out.IsEmpty()) Out = TEXT("Vehicle");
  if (Out[0] >= '0' && Out[0] <= '9') Out = TEXT("V_") + Out;
  return Out;
}

static UStaticMesh* ImportStaticMesh(const FString& FilePath,
                                     const FString& ContentPath,
                                     const FString& AssetName,
                                     const FVehicleImportSpec& Spec)
{
  UAssetImportTask* Task = NewObject<UAssetImportTask>();
  Task->Filename        = FilePath;
  Task->DestinationPath = ContentPath;
  Task->DestinationName = SanitizeAssetName(AssetName);





  Task->bSave           = false;
  Task->bAutomated      = true;
  Task->bReplaceExisting = true;

  IAssetTools& AssetTools =
    FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();
  AssetTools.ImportAssetTasks({ Task });

  TArray<UObject*> Imported = Task->GetObjects();
  if (Imported.Num() == 0)
    return nullptr;
  UStaticMesh* Mesh = Cast<UStaticMesh>(Imported[0]);
  if (!Mesh) return nullptr;







  if (Mesh->GetNumSourceModels() > 0)
  {
    FStaticMeshSourceModel& SM = Mesh->GetSourceModel(0);
    if (!SM.BuildSettings.BuildScale3D.Equals(FVector(1.0)))
    {
      SM.BuildSettings.BuildScale3D = FVector(1.0);
      Mesh->Build(false);
      Mesh->MarkPackageDirty();
    }
  }
  return Mesh;
}

static TSubclassOf<UChaosVehicleWheel> CreateWheelBlueprint(
    const FString& ContentPath,
    const FString& Name,
    const FWheelImportSpec& W)
{
  IAssetTools& AssetTools =
    FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();

  UBlueprintFactory* Factory = NewObject<UBlueprintFactory>();
  Factory->ParentClass = UChaosVehicleWheel::StaticClass();

  UObject* Asset = AssetTools.CreateAsset(Name, ContentPath,
                                          UBlueprint::StaticClass(), Factory);
  UBlueprint* BP = Cast<UBlueprint>(Asset);
  if (!BP || !BP->GeneratedClass) return nullptr;







  UChaosVehicleWheel* Defaults =
    Cast<UChaosVehicleWheel>(BP->GeneratedClass->ClassDefaultObject);
  if (Defaults)
  {
    Defaults->WheelRadius        = W.Radius;
    Defaults->MaxSteerAngle      = W.MaxSteerAngle;
    Defaults->MaxBrakeTorque     = W.MaxBrakeTorque;
    Defaults->SuspensionMaxRaise = W.SuspMaxRaise;
    Defaults->SuspensionMaxDrop  = W.SuspMaxDrop;
  }
  return TSubclassOf<UChaosVehicleWheel>(BP->GeneratedClass);
}

FString FVehicleImporterServer::ProcessSpec(const FVehicleImportSpec& Spec)
{
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: enter (vehicle=%s mesh=%s)"),
         *Spec.VehicleName, *Spec.MeshFilePath);


  FString ContentRoot = Spec.ContentPath;
  if (ContentRoot.EndsWith(TEXT("/")))
    ContentRoot.RemoveFromEnd(TEXT("/"));
  const FString VehicleContentPath = ContentRoot / Spec.VehicleName;






  if (UEditorAssetLibrary::DoesDirectoryExist(VehicleContentPath))
  {
    UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: %s already exists - deleting before re-import"),
           *VehicleContentPath);
    UEditorAssetLibrary::DeleteDirectory(VehicleContentPath);
  }


  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: step 1/5 - importing body mesh"));
  UStaticMesh* BodyMesh = ImportStaticMesh(
    Spec.MeshFilePath, VehicleContentPath,
    Spec.VehicleName + TEXT("_body"), Spec);
  if (!BodyMesh)
    return MakeResponse(false, TEXT(""), TEXT("Failed to import mesh: ") + Spec.MeshFilePath);
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: body mesh imported OK"));


  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: step 2/5 - creating 4 wheel blueprints"));
  auto MakeWheelBP = [&](const FString& Suffix, const FWheelImportSpec& W)
    -> TSubclassOf<UChaosVehicleWheel>
  {
    UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec:   wheel %s …"), *Suffix);
    auto R = CreateWheelBlueprint(
      VehicleContentPath,
      Spec.VehicleName + TEXT("_Wheel_") + Suffix,
      W);
    UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec:   wheel %s done (%s)"),
           *Suffix, R ? TEXT("ok") : TEXT("FAILED"));
    return R;
  };

  TSubclassOf<UChaosVehicleWheel> WheelFL = MakeWheelBP(TEXT("FLW"), Spec.WheelFL);
  TSubclassOf<UChaosVehicleWheel> WheelFR = MakeWheelBP(TEXT("FRW"), Spec.WheelFR);
  TSubclassOf<UChaosVehicleWheel> WheelRL = MakeWheelBP(TEXT("RLW"), Spec.WheelRL);
  TSubclassOf<UChaosVehicleWheel> WheelRR = MakeWheelBP(TEXT("RRW"), Spec.WheelRR);

  if (!WheelFL || !WheelFR || !WheelRL || !WheelRR)
    return MakeResponse(false, TEXT(""), TEXT("Failed to create wheel blueprints"));



  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: step 3/5 - loading base BP %s"),
         *Spec.BaseVehicleBP);
  UClass* BaseClass = nullptr;
  if (!Spec.BaseVehicleBP.IsEmpty())
  {
    UObject* BPObj = UEditorAssetLibrary::LoadAsset(Spec.BaseVehicleBP);
    UBlueprint* BPAsset = Cast<UBlueprint>(BPObj);
    if (BPAsset && BPAsset->GeneratedClass)
      BaseClass = BPAsset->GeneratedClass;
  }
  if (!BaseClass)
  {



    UObject* TplObj = UEditorAssetLibrary::LoadAsset(
      TEXT("/Game/Carla/Blueprints/USDImportTemplates/BaseUSDImportVehicle"));
    if (UBlueprint* TplBP = Cast<UBlueprint>(TplObj))
      BaseClass = TplBP->GeneratedClass;
  }
  if (!BaseClass)
    return MakeResponse(false, TEXT(""),
      TEXT("Could not resolve base vehicle class - set base_vehicle_bp in the spec"));





  FMergedVehicleMeshParts Parts;
  Parts.Body = BodyMesh;
  Parts.Anchors.WheelFL = FVector(Spec.WheelFL.X, Spec.WheelFL.Y, Spec.WheelFL.Z);
  Parts.Anchors.WheelFR = FVector(Spec.WheelFR.X, Spec.WheelFR.Y, Spec.WheelFR.Z);
  Parts.Anchors.WheelRL = FVector(Spec.WheelRL.X, Spec.WheelRL.Y, Spec.WheelRL.Z);
  Parts.Anchors.WheelRR = FVector(Spec.WheelRR.X, Spec.WheelRR.Y, Spec.WheelRR.Z);

  FWheelTemplates WheelTemplates;
  WheelTemplates.WheelFL = WheelFL;
  WheelTemplates.WheelFR = WheelFR;
  WheelTemplates.WheelRL = WheelRL;
  WheelTemplates.WheelRR = WheelRR;









  USkeletalMeshComponent* SkelComp = nullptr;
  auto TakeIfBetter = [&](USkeletalMeshComponent* C) {
    if (!C) return;
    if (C->GetSkeletalMeshAsset()) { SkelComp = C; return; }
    if (!SkelComp) SkelComp = C;
  };
  bool TriedFallback = false;

InspectBaseClass:
  AActor* TemplateDefault = BaseClass->GetDefaultObject<AActor>();
  SkelComp = nullptr;


  if (TemplateDefault)
  {
    TArray<UActorComponent*> Components;
    TemplateDefault->GetComponents(Components);
    UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: CDO has %d components on %s"),
           Components.Num(), *BaseClass->GetName());
    for (UActorComponent* C : Components)
    {
      if (!C) continue;
      UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec:   CDO comp: %s (%s)"),
             *C->GetName(), *C->GetClass()->GetName());
      TakeIfBetter(Cast<USkeletalMeshComponent>(C));
    }
  }


  if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
  {
    UClass* WalkClass = BaseClass;
    while (WalkClass)
    {
      UBlueprint* WalkBP = Cast<UBlueprint>(WalkClass->ClassGeneratedBy);
      UBlueprintGeneratedClass* OwnerGen = Cast<UBlueprintGeneratedClass>(WalkBP ? WalkBP->GeneratedClass : nullptr);
      if (WalkBP && WalkBP->SimpleConstructionScript && OwnerGen)
      {
        const auto& Nodes = WalkBP->SimpleConstructionScript->GetAllNodes();
        UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: SCS for %s has %d node(s)"),
               *WalkClass->GetName(), Nodes.Num());
        for (USCS_Node* Node : Nodes)
        {
          if (!Node) continue;
          UActorComponent* Tmpl = Node->GetActualComponentTemplate(OwnerGen);
          UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec:   SCS node: %s (template=%s)"),
                 *Node->GetVariableName().ToString(),
                 Tmpl ? *Tmpl->GetClass()->GetName() : TEXT("null"));
          TakeIfBetter(Cast<USkeletalMeshComponent>(Tmpl));
        }
        if (SkelComp && SkelComp->GetSkeletalMeshAsset()) break;
      }
      WalkClass = WalkClass->GetSuperClass();
    }
  }







  if (!SkelComp || !SkelComp->GetSkeletalMeshAsset())
  {
    UWorld* SpawnWorld = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
    if (SpawnWorld)
    {
      FActorSpawnParameters Params;
      Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      Params.ObjectFlags = RF_Transient;
      AActor* Tmp = SpawnWorld->SpawnActor<AActor>(BaseClass, FTransform::Identity, Params);
      UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: spawned %s temporarily for inspection (%s)"),
             *BaseClass->GetName(), Tmp ? TEXT("ok") : TEXT("FAILED"));
      if (Tmp)
      {
        TArray<USkeletalMeshComponent*> SMCs;
        Tmp->GetComponents<USkeletalMeshComponent>(SMCs);
        UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec:   spawned actor has %d SkeletalMeshComponent(s)"), SMCs.Num());
        for (USkeletalMeshComponent* C : SMCs) TakeIfBetter(C);
        Tmp->Destroy();
      }
    }
  }

  USkeletalMesh*  SkelMesh    = SkelComp ? SkelComp->GetSkeletalMeshAsset() : nullptr;
  UPhysicsAsset*  PhysAsset   = SkelMesh  ? SkelMesh->GetPhysicsAsset()      : nullptr;
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: step 4/5 - SkelMesh=%s PhysAsset=%s"),
         SkelMesh  ? *SkelMesh->GetName()  : TEXT("null"),
         PhysAsset ? *PhysAsset->GetName() : TEXT("null"));

  if ((!SkelMesh || !PhysAsset) && !TriedFallback)
  {
    static const TCHAR* kFallbackBP =
      TEXT("/Game/Carla/Blueprints/USDImportTemplates/BaseUSDImportVehicle");
    UE_LOG(LogCarlaTools, Warning,
           TEXT("VI.ProcessSpec: %s yielded no SkelMesh/PhysAsset (production BPs are static-mesh "
                "rigs and can't be used as templates) - falling back to %s and re-inspecting"),
           *BaseClass->GetName(), kFallbackBP);
    UObject* TplObj = UEditorAssetLibrary::LoadAsset(kFallbackBP);
    if (UBlueprint* TplBP = Cast<UBlueprint>(TplObj))
    {
      if (TplBP->GeneratedClass)
      {
        BaseClass = TplBP->GeneratedClass;
        TriedFallback = true;
        goto InspectBaseClass;
      }
    }
  }

  if (!SkelMesh || !PhysAsset)
    return MakeResponse(false, TEXT(""),
      FString::Printf(TEXT("Base vehicle blueprint has no skeletal mesh / physics asset (BP=%s SkelMesh=%s PhysAsset=%s)"),
        *Spec.BaseVehicleBP,
        SkelMesh ? *SkelMesh->GetName() : TEXT("null"),
        PhysAsset ? *PhysAsset->GetName() : TEXT("null")));


  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: step 5/5 - generating vehicle blueprint"));
  UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
  if (!World)
    return MakeResponse(false, TEXT(""), TEXT("No editor world available"));

  const FString BPPath = VehicleContentPath / (TEXT("BP_") + Spec.VehicleName);

  UUSDImporterWidget::GenerateNewVehicleBlueprint(
    World, BaseClass, SkelMesh, PhysAsset, BPPath, Parts, WheelTemplates);







  TArray<FString> ToSave;
  ToSave.Add(VehicleContentPath / (Spec.VehicleName + TEXT("_body")));
  ToSave.Add(VehicleContentPath / (Spec.VehicleName + TEXT("_Wheel_FLW")));
  ToSave.Add(VehicleContentPath / (Spec.VehicleName + TEXT("_Wheel_FRW")));
  ToSave.Add(VehicleContentPath / (Spec.VehicleName + TEXT("_Wheel_RLW")));
  ToSave.Add(VehicleContentPath / (Spec.VehicleName + TEXT("_Wheel_RRW")));
  ToSave.Add(BPPath);
  int32 Saved = 0;
  for (const FString& Path : ToSave)
  {
    if (UEditorAssetLibrary::SaveAsset(Path, false))
      ++Saved;
    else
      UE_LOG(LogCarlaTools, Warning, TEXT("VI.ProcessSpec: SaveAsset failed for %s"), *Path);
  }
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpec: persisted %d/%d assets to disk"),
         Saved, ToSave.Num());

  return MakeResponse(true, BPPath, TEXT(""));
}





FString FVehicleImporterServer::ProcessSpawn(const FSpawnRequest& Req)
{
  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpawn: enter (asset=%s loc=%s yaw=%.1f)"),
         *Req.AssetPath, *Req.Loc.ToString(), Req.Yaw);

  if (Req.AssetPath.IsEmpty())
    return MakeResponse(false, TEXT(""), TEXT("spawn: asset_path missing"));







  auto fixupClassPath = [](const FString& In) -> FString {



    if (In.EndsWith(TEXT("_C"))) return In;
    int32 Dot = INDEX_NONE; In.FindLastChar(TEXT('.'), Dot);
    int32 Slash = INDEX_NONE; In.FindLastChar(TEXT('/'), Slash);
    if (Dot == INDEX_NONE || Dot < Slash)
    {
      const FString Leaf = (Slash == INDEX_NONE) ? In : In.RightChop(Slash + 1);
      return In + TEXT(".") + Leaf + TEXT("_C");
    }
    return In + TEXT("_C");
  };

  UClass* Cls = nullptr;
  FString AttemptedPaths;

  {
    AttemptedPaths += Req.AssetPath;
    UObject* Loaded = UEditorAssetLibrary::LoadAsset(Req.AssetPath);
    if (UBlueprint* BP = Cast<UBlueprint>(Loaded))
      Cls = BP->GeneratedClass;
    else if (UClass* DirectClass = Cast<UClass>(Loaded))
      Cls = DirectClass;
  }

  if (!Cls)
  {
    const FString Fixed = fixupClassPath(Req.AssetPath);
    AttemptedPaths += TEXT(" | ") + Fixed;
    Cls = LoadClass<AActor>(nullptr, *Fixed);
  }


  if (!Cls)
  {
    const FString Fixed = fixupClassPath(Req.AssetPath);
    Cls = Cast<UClass>(StaticLoadObject(UClass::StaticClass(), nullptr, *Fixed));
  }

  if (!Cls || !Cls->IsChildOf(AActor::StaticClass()))
    return MakeResponse(false, TEXT(""),
      FString::Printf(TEXT("spawn: could not resolve actor class. Tried: %s"),
                      *AttemptedPaths));

  UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
  if (!World)
    return MakeResponse(false, TEXT(""), TEXT("spawn: no editor world available"));

  FActorSpawnParameters Params;
  Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor* Spawned = World->SpawnActor<AActor>(
      Cls, FTransform(FRotator(0.f, Req.Yaw, 0.f), Req.Loc), Params);
  if (!Spawned)
    return MakeResponse(false, TEXT(""),
      FString::Printf(TEXT("spawn: SpawnActor returned null for %s"), *Cls->GetName()));

  UE_LOG(LogCarlaTools, Display, TEXT("VI.ProcessSpawn: spawned %s (label=%s)"),
         *Cls->GetName(), *Spawned->GetActorLabel());
  return MakeResponse(true, Spawned->GetActorLabel(), TEXT(""));
}
