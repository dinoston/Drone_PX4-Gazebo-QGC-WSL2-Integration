// Copyright Epic Games, Inc. All Rights Reserved.

#include "FlyingNPCPawn.h"

#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/PawnMovementComponent.h"

AFlyingNPCPawn::AFlyingNPCPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	CollisionSphere = CreateDefaultSubobject<USphereComponent>(TEXT("CollisionSphere"));
	SetRootComponent(CollisionSphere);
	CollisionSphere->InitSphereRadius(100.0f);
	CollisionSphere->SetCollisionProfileName(TEXT("Pawn"));
	CollisionSphere->SetGenerateOverlapEvents(false);
	CollisionSphere->SetNotifyRigidBodyCollision(true);

	DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
	DroneMesh->SetupAttachment(CollisionSphere);
	// The simple sphere owns physical collision; the visual mesh stays query-free.
	// 단순 구체가 물리 충돌을 담당하고 시각 메시 자체의 충돌은 비활성화합니다.
	DroneMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	FlightMovement = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("FlightMovement"));
	FlightMovement->SetUpdatedComponent(CollisionSphere);
	FlightMovement->TurningBoost = 8.0f;

	AutoPossessAI = EAutoPossessAI::PlacedInWorldOrSpawned;
	Tags.Add(TEXT("CivilianDrone"));
}

void AFlyingNPCPawn::BeginPlay()
{
	Super::BeginPlay();

	ApplyMovementSettings();
	CollisionSphere->OnComponentHit.AddDynamic(this, &AFlyingNPCPawn::HandleCollisionHit);

	if (bStartPatrolOnBeginPlay)
	{
		StartPatrol();
	}
}

void AFlyingNPCPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	ApplyMovementSettings();
	AvoidanceCommitRemaining = FMath::Max(0.0f, AvoidanceCommitRemaining - DeltaSeconds);

	if (!bPatrolActive || FlightMovement == nullptr)
	{
		return;
	}

	FVector TargetLocation;
	if (!ResolveTargetLocation(TargetLocation))
	{
		StopPatrol();
		return;
	}

	const FVector ToTarget = TargetLocation - GetActorLocation();
	if (ToTarget.Size() <= AcceptanceRadiusCm)
	{
		if (bHasDirectTarget)
		{
			ClearDirectTarget();
		}
		else
		{
			AdvancePatrolPoint();
		}

		if (!bPatrolActive || !ResolveTargetLocation(TargetLocation))
		{
			return;
		}
	}

	const FVector DesiredDirection = (TargetLocation - GetActorLocation()).GetSafeNormal();
	if (DesiredDirection.IsNearlyZero())
	{
		return;
	}

	const FVector SteeringDirection = ComputeSteeringDirection(DesiredDirection, DeltaSeconds);
	FlightMovement->AddInputVector(SteeringDirection, true);

	// Keep the drone level and smoothly rotate only around yaw.
	// 드론의 수평 자세를 유지하고 Yaw 방향만 부드럽게 회전합니다.
	const FVector FlatDirection(SteeringDirection.X, SteeringDirection.Y, 0.0f);
	if (!FlatDirection.IsNearlyZero())
	{
		const FRotator TargetRotation = FlatDirection.Rotation();
		const FRotator LevelTargetRotation(0.0f, TargetRotation.Yaw, 0.0f);
		SetActorRotation(FMath::RInterpTo(
			GetActorRotation(),
			LevelTargetRotation,
			DeltaSeconds,
			RotationInterpSpeed));
	}
}

UPawnMovementComponent* AFlyingNPCPawn::GetMovementComponent() const
{
	return FlightMovement;
}

void AFlyingNPCPawn::StartPatrol()
{
	if (bHasDirectTarget || PatrolPoints.Num() > 0)
	{
		PatrolPointIndex = FMath::Clamp(PatrolPointIndex, 0, FMath::Max(0, PatrolPoints.Num() - 1));
		bPatrolActive = true;
	}
}

void AFlyingNPCPawn::StopPatrol()
{
	bPatrolActive = false;
	if (FlightMovement != nullptr)
	{
		FlightMovement->StopMovementImmediately();
	}
}

void AFlyingNPCPawn::SetDirectTarget(const FVector& WorldLocation)
{
	DirectTargetLocation = WorldLocation;
	bHasDirectTarget = true;
	bPatrolActive = true;
}

void AFlyingNPCPawn::ClearDirectTarget()
{
	bHasDirectTarget = false;
	bPatrolActive = PatrolPoints.Num() > 0;
}

bool AFlyingNPCPawn::ResolveTargetLocation(FVector& OutTargetLocation) const
{
	if (bHasDirectTarget)
	{
		OutTargetLocation = DirectTargetLocation;
		return true;
	}

	if (!PatrolPoints.IsValidIndex(PatrolPointIndex) || PatrolPoints[PatrolPointIndex] == nullptr)
	{
		return false;
	}

	OutTargetLocation = PatrolPoints[PatrolPointIndex]->GetActorLocation();
	return true;
}

void AFlyingNPCPawn::AdvancePatrolPoint()
{
	const int32 PointCount = PatrolPoints.Num();
	if (PointCount <= 0)
	{
		StopPatrol();
		return;
	}

	if (bRandomizePatrol && PointCount > 1)
	{
		int32 NextIndex = PatrolPointIndex;
		while (NextIndex == PatrolPointIndex)
		{
			NextIndex = FMath::RandRange(0, PointCount - 1);
		}
		PatrolPointIndex = NextIndex;
		return;
	}

	if (PatrolPointIndex + 1 < PointCount)
	{
		++PatrolPointIndex;
	}
	else if (bLoopPatrol)
	{
		PatrolPointIndex = 0;
	}
	else
	{
		StopPatrol();
	}
}

FVector AFlyingNPCPawn::ComputeSteeringDirection(const FVector& DesiredDirection, float DeltaSeconds)
{
	(void)DeltaSeconds;
	if (AvoidanceCommitRemaining > 0.0f && !CommittedAvoidanceDirection.IsNearlyZero())
	{
		return (DesiredDirection + CommittedAvoidanceDirection * AvoidanceStrength).GetSafeNormal();
	}

	const float ForwardClearance = TraceClearance(
		DesiredDirection,
		AvoidanceTraceDistanceCm,
		bDrawAvoidanceDebug);
	if (ForwardClearance >= AvoidanceTraceDistanceCm * 0.98f)
	{
		CommittedAvoidanceDirection = FVector::ZeroVector;
		return DesiredDirection;
	}

	FVector FlatForward(DesiredDirection.X, DesiredDirection.Y, 0.0f);
	if (!FlatForward.Normalize())
	{
		FlatForward = GetActorForwardVector();
		FlatForward.Z = 0.0f;
		FlatForward.Normalize();
	}
	const FVector RightDirection = FVector::CrossProduct(FVector::UpVector, FlatForward).GetSafeNormal();

	// Evaluate stable escape directions around the blocked forward path.
	// 막힌 전방 경로 주변의 안정적인 회피 방향을 비교합니다.
	const TArray<FVector> Candidates = {
		RightDirection,
		-RightDirection,
		FVector::UpVector,
		-FVector::UpVector,
		(RightDirection + FVector::UpVector).GetSafeNormal(),
		(-RightDirection + FVector::UpVector).GetSafeNormal(),
		(RightDirection - FVector::UpVector).GetSafeNormal(),
		(-RightDirection - FVector::UpVector).GetSafeNormal()
	};

	float BestScore = -1.0f;
	FVector BestDirection = -DesiredDirection;
	for (const FVector& Candidate : Candidates)
	{
		const float Clearance = TraceClearance(Candidate, AvoidanceTraceDistanceCm, bDrawAvoidanceDebug);
		// Prefer open space while retaining a small bias toward the destination.
		// 넓게 열린 공간을 우선하되 목적지 방향에 약간의 가중치를 둡니다.
		const float ProgressBias = FVector::DotProduct(Candidate, DesiredDirection)
			* AvoidanceTraceDistanceCm * 0.15f;
		const float Score = Clearance + ProgressBias;
		if (Score > BestScore)
		{
			BestScore = Score;
			BestDirection = Candidate;
		}
	}

	CommittedAvoidanceDirection = BestDirection;
	AvoidanceCommitRemaining = AvoidanceCommitSeconds;
	return (DesiredDirection + BestDirection * AvoidanceStrength).GetSafeNormal();
}

float AFlyingNPCPawn::TraceClearance(const FVector& Direction, float DistanceCm, bool bDebug) const
{
	UWorld* World = GetWorld();
	if (World == nullptr || Direction.IsNearlyZero())
	{
		return 0.0f;
	}

	const FVector Start = GetActorLocation();
	const FVector End = Start + Direction.GetSafeNormal() * DistanceCm;
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(FlyingNPCObstacleProbe), false, this);
	QueryParams.AddIgnoredActor(this);
	FHitResult Hit;
	const bool bHit = World->SweepSingleByChannel(
		Hit,
		Start,
		End,
		FQuat::Identity,
		ObstacleTraceChannel,
		FCollisionShape::MakeSphere(AvoidanceProbeRadiusCm),
		QueryParams);

	if (bDebug)
	{
		const FColor Color = bHit ? FColor::Red : FColor::Green;
		DrawDebugLine(World, Start, bHit ? Hit.Location : End, Color, false, 0.05f, 0, 2.0f);
		DrawDebugSphere(
			World,
			bHit ? Hit.Location : End,
			AvoidanceProbeRadiusCm,
			12,
			Color,
			false,
			0.05f);
	}

	return bHit ? FMath::Max(0.0f, Hit.Distance) : DistanceCm;
}

void AFlyingNPCPawn::ApplyMovementSettings()
{
	if (FlightMovement == nullptr)
	{
		return;
	}

	FlightMovement->MaxSpeed = MaxFlightSpeedCmPerSecond;
	FlightMovement->Acceleration = AccelerationCmPerSecondSquared;
	FlightMovement->Deceleration = DecelerationCmPerSecondSquared;
}

void AFlyingNPCPawn::HandleCollisionHit(
	UPrimitiveComponent* HitComponent,
	AActor* OtherActor,
	UPrimitiveComponent* OtherComponent,
	FVector NormalImpulse,
	const FHitResult& Hit)
{
	if (OtherActor == nullptr || OtherActor == this)
	{
		return;
	}

	// Stop pushing the surface and commit briefly to its outward normal.
	// 충돌면을 계속 밀지 않고 표면 바깥쪽 법선 방향으로 잠시 회피합니다.
	if (FlightMovement != nullptr)
	{
		FlightMovement->StopMovementImmediately();
	}
	CommittedAvoidanceDirection = Hit.ImpactNormal.IsNearlyZero()
		? Hit.Normal.GetSafeNormal()
		: Hit.ImpactNormal.GetSafeNormal();
	AvoidanceCommitRemaining = FMath::Max(AvoidanceCommitSeconds, 1.0f);
}
